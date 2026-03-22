/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Rejestrator 4ch ADC + FFT
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h" // Główny plik nagłówkowy wygenerowany przez STM32CubeMX (zawiera definicje pinów i bibliotekę HAL)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // Biblioteka do operacji na pamięci (memcpy)
#include <stdio.h>  // Biblioteka standardowa wejścia/wyjścia (sprintf)
#include <stdarg.h> // Biblioteka do obsługi zmiennej liczby argumentów (va_list dla USART_fsend)
#include <stdlib.h> // Biblioteka standardowa (zawiera m.in. funkcje konwersji, choć teraz rzadziej używana)
#include <math.h>   // Biblioteka matematyczna niezbędna do obliczeń FFT (sin, cos, sqrt, log10)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNELS        4    // Liczba kanałów przetwornika ADC, które skanujemy
#define SAMPLES_PER_CH      1024 // Liczba próbek na jeden kanał (1024 to optymalna wartość dla FFT, potęga dwójki)
#define DMA_BUF_LEN         (SAMPLES_PER_CH * ADC_CHANNELS * 2) // Całkowity rozmiar bufora DMA. Razy 2, ponieważ używamy trybu Ping-Pong (Double Buffering)
#define PI                  3.14159265359 // Stała Pi używana w algorytmie FFT
#define HISTORY_SIZE        10   // Liczba elementów w buforze historii pomiarów
#define USART_RXBUF_LEN     256  // Rozmiar bufora kołowego do odbierania danych z UART
#define USART_TXBUFF_LEN    512  // Rozmiar bufora pomocniczego do wysyłania tekstów (USART_fsend)

// Makro (obecnie nieużywane w trybie przerwaniowym), które obliczało pozycję głowy DMA
#define DMA_RX_HEAD (USART_RXBUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1; // Uchwyt do konfiguracji i sterowania przetwornikiem ADC1
TIM_HandleTypeDef htim2; // Uchwyt do konfiguracji Timera 2 (wyzwala pomiary ADC)
DMA_HandleTypeDef hdma_adc1; // Uchwyt do kanału DMA obsługującego transfer z ADC
UART_HandleTypeDef huart2; // Uchwyt do interfejsu UART2 (komunikacja z PC)
DMA_HandleTypeDef hdma_usart2_rx; // Uchwyt DMA dla UART RX (nieużywany w nowej wersji na przerwania)

/* USER CODE BEGIN PV */
// --- BUFORY ---
uint16_t adc_dma_buffer[DMA_BUF_LEN]; // Główny, przeplatany bufor, do którego DMA wrzuca surowe wyniki pomiarów ADC
float ch0_fft_input[SAMPLES_PER_CH]; // Bufor na część rzeczywistą (Re) dla kanału 0 do obliczeń FFT
uint16_t ch1_buffer[SAMPLES_PER_CH]; // Odseparowany bufor na próbki z kanału 1 (do wyliczenia amplitudy AC)
uint16_t ch2_buffer[SAMPLES_PER_CH]; // Odseparowany bufor na próbki z kanału 2 (DC)
uint16_t ch3_buffer[SAMPLES_PER_CH]; // Odseparowany bufor na próbki z kanału 3 (DC)

volatile uint8_t flag_half_transfer = 0; // Flaga ustawiana w przerwaniu, gdy DMA zapełni PIERWSZĄ połowę bufora
volatile uint8_t flag_full_transfer = 0; // Flaga ustawiana w przerwaniu, gdy DMA zapełni DRUGĄ połowę bufora

// --- STRUKTURA WYNIKÓW ---
typedef struct {
    uint32_t ch0_dom_freq;  // Obliczona dominująca częstotliwość (Hz) z FFT
    uint32_t ch0_amplitude; // Obliczona amplituda prążka dominującego z FFT
    uint16_t ch1_avg, ch1_med; // ch1_avg = Amplituda Peak-to-Peak (AC), ch1_med = Offset DC
    uint16_t ch2_avg, ch2_med; // Średnia i mediana sygnału stałego DC na kanale 2
    uint16_t ch3_avg, ch3_med; // Średnia i mediana sygnału stałego DC na kanale 3
} AnalysisResult_t; // Definicja typu struktury przechowującej komplet wyników

AnalysisResult_t current_results; // Zmienna trzymająca najświeższe wyniki pomiarów
AnalysisResult_t history_buffer[HISTORY_SIZE]; // Tablica archiwum (historia 10 ostatnich pomiarów)
uint8_t history_idx = 0; // Wskaźnik, w którym miejscu historii aktualnie zapisujemy

// --- LOGOWANIE I UART ---
uint32_t measure_interval_ms = 1000; // Czas w milisekundach określający, jak często zapisywać dane do historii
uint32_t last_log_time = 0; // Zmienna przechowująca czas (z HAL_GetTick) ostatniego zapisu do historii
uint32_t stat_rx_count = 0; // Licznik poprawnie odebranych ramek (do debugowania)
uint32_t stat_tx_count = 0; // Licznik wysłanych odpowiedzi (do debugowania)

uint8_t USART_TxBuf[USART_TXBUFF_LEN]; // Bufor nadawczy
uint8_t USART_RxBuf[USART_RXBUF_LEN];  // Główny bufor kołowy (Ring Buffer) odbiornika UART
__IO int USART_TX_Empty=0; // Zmienne pomocnicze (nieużywane w tej wersji)
__IO int USART_TX_Busy=0;  // Zmienne pomocnicze (nieużywane w tej wersji)
__IO int USART_RX_Empty=0; // "Ogon" bufora RX - wskazuje, do którego miejsca main() odczytał dane
volatile uint32_t USART_RX_Busy = 0; // "Głowa" bufora RX - wskazuje, gdzie przerwanie wpisało ostatni znak

// Parser
typedef enum { FRAME_WAIT_START, FRAME_READ_PAYLOAD } FrameState_t; // Stany maszyny stanów parsera
FrameState_t parser_state = FRAME_WAIT_START; // Aktualny stan maszyny (na start: czekanie na znak '<')
uint16_t parser_idx = 0; // Wskaźnik określający, na którym znaku budowanej ramki jesteśmy
char raw_payload[300]; // Bufor tymczasowy do sklejania odkodowanej ramki z UART
char cmd_data[256]; // Zmienna pomocnicza
char recv_crc_str[5]; // Zmienna pomocnicza do trzymania 4 znaków CRC z ramki jako string
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// obsluga buforow kolowych i przerwan

// Funkcja wysyłająca tekst przez UART używając formatowania jak w printf()
void USART_fsend(char* format,...) {
    char tmp_rs[512]; // Tymczasowy bufor na sformatowany tekst
    va_list arglist; // Lista argumentów dla funkcji ze zmienną liczbą parametrów
    va_start(arglist,format); // Inicjalizacja listy
    vsprintf(tmp_rs,format,arglist); // Sklejenie stringa z podanymi zmiennymi
    va_end(arglist); // Zwolnienie listy
    HAL_UART_Transmit(&huart2, (uint8_t*)tmp_rs, strlen(tmp_rs), 100); // Wysłanie gotowego tekstu przez HAL (z timeoutem 100ms)
}

// Proste CRC (XOR + stała) do zabezpieczenia transmisji
uint16_t Calculate_CRC(char* data, uint16_t len) {
    uint16_t sum = 0; // Zmienna trzymająca sumę
    for (int i = 0; i < len; i++) sum += (uint8_t)data[i]; // Dodanie wartości ASCII/HEX każdego znaku
    return (sum ^ 0x1021); // Nałożenie operacji logicznej XOR z wielomianem 0x1021
}

// funkcja do wysyłania surowych bajtów
void Send_Binary(uint8_t* payload, uint16_t payload_len) {
    uint8_t response_buffer[512]; // Większy bufor w razie konieczności rozrostu po kodowaniu
    uint8_t crc_input[260]; // Bufor na dane do obliczenia CRC

    // 1. Składamy to, co wchodzi do CRC (ID + DANE)
    crc_input[0] = 'S'; crc_input[1] = 'T'; crc_input[2] = 'P'; crc_input[3] = 'C'; // Nagłówek ID Nadawcy i Odbiorcy
    memcpy(&crc_input[4], payload, payload_len); // Skopiowanie binarnych danych (bez sprawdzania znaków końca \0)

    uint16_t total_crc_len = 4 + payload_len; // Całkowita długość tego, co podlega CRC

    // 2. Liczymy CRC na CZYSTYCH, niezakodowanych danych
    uint16_t crc = Calculate_CRC((char*)crc_input, total_crc_len); // Obliczenie weryfikatora dla paczki

    // 3. Budujemy finalną ramkę (Z KODOWANIEM ZNAKÓW - Byte Stuffing)
    uint16_t out_idx = 0; // Indeks wyjściowego bufora UART
    response_buffer[out_idx++] = '<'; // Wstawienie znacznika początku ramki

    for(uint16_t i = 0; i < total_crc_len; i++) { // Iteracja przez każdy bajt z oryginalnej paczki
        uint8_t current_byte = crc_input[i]; // Pobranie bajtu

        // KODOWANIE (Byte Stuffing)
        if (current_byte == '\\') { // Jeśli dane to znak 'ukośnik'
            response_buffer[out_idx++] = '\\'; // Wstaw znak ucieczki
            response_buffer[out_idx++] = '0';  // Wstaw '0' jako kod ukośnika
        }
        else if (current_byte == '<') { // Jeśli w danych pojawił się bajt równy 'początek ramki'
            response_buffer[out_idx++] = '\\'; // Wstaw znak ucieczki
            response_buffer[out_idx++] = '1';  // Wstaw '1' jako kod początku ramki
        }
        else if (current_byte == '>') { // Jeśli w danych pojawił się bajt równy 'koniec ramki'
            response_buffer[out_idx++] = '\\'; // Wstaw znak ucieczki
            response_buffer[out_idx++] = '2';  // Wstaw '2' jako kod końca ramki
        }
        else { // Jeśli bajt jest "zwykłą" liczbą
            // Zwykły bajt
            response_buffer[out_idx++] = current_byte; // wstawienie go do ramki wyjściowej
        }
    }

    // 4. Dopisujemy 4 znaki CRC i znak końca (Tego już nie musimy kodować, to zawsze ASCII)
    sprintf((char*)&response_buffer[out_idx], "%04X>\r\n", crc); // Zapis CRC jako 4 znaki HEX, zamknięcie ramki '>' i przejście do nowej linii
    out_idx += 7; // Przesunięcie indeksu o 4(CRC) + 1('>') + 1('\r') + 1('\n')

    // 5. Wysyłka
    HAL_UART_Transmit(&huart2, response_buffer, out_idx, 100); // Wypchnięcie gotowej ramki binarnej do PC
    stat_tx_count++; // Podbicie licznika wysłanych ramek
}

// Algorytm sortowania bąbelkowego (używany do znalezienia mediany)
void BubbleSort(uint16_t* arr, int n) {
    for (int i = 0; i < n - 1; i++) { // Pętla po całej tablicy
        for (int j = 0; j < n - i - 1; j++) { // Pętla porównująca sąsiadów
            if (arr[j] > arr[j + 1]) { // Jeśli lewy jest większy od prawego...
                uint16_t temp = arr[j]; // ...to zamień je miejscami (swap)
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// Algorytm FFT (Cooley-Tukey Radix-2 DIT) do zamiany sygnału w czasie na widmo częstotliwościowe
void FFT(float *rex, float *imx, int n) {
    int m = (int)(log10(n) / log10(2)); // Obliczenie liczby etapów (dla N=1024 to 10 etapów)
    int i, j, k, n1, n2; // Zmienne pomocnicze do pętli
    float c, s, e, a, t1, t2; // Zmienne do trygonometrii (sin/cos) i tymczasowych obliczeń
    j = 0; n2 = n / 2; // Inicjalizacja do odwracania bitów (Bit Reversal)

    // 1. Bit Reversal (Tasowanie próbek do odpowiedniej kolejności przed FFT)
    for (i = 1; i < n - 1; i++) {
        n1 = n2;
        while (j >= n1) { j = j - n1; n1 = n1 / 2; }
        j = j + n1;
        if (i < j) { // Jeśli indeks odwrotny jest większy, zamień pozycje
            t1 = rex[i]; rex[i] = rex[j]; rex[j] = t1; // Zamiana części Rzeczywistej
            t1 = imx[i]; imx[i] = imx[j]; imx[j] = t1; // Zamiana części Urojonej
        }
    }

    // 2. Właściwe obliczenia FFT (tzw. "Motylki" / Butterfly Operations)
    n1 = 0; n2 = 1;
    for (i = 0; i < m; i++) { // Pętla po etapach logarytmicznych
        n1 = n2; n2 = n2 + n2; // Zwiększanie rozmiaru "motylka" dwukrotnie
        e = -3.14159265359 / n1; a = 0.0; // Ustalenie kąta dla współczynników
        for (j = 0; j < n1; j++) { // Pętla po elementach wewnątrz grupy
            c = cos(a); s = sin(a); a = a + e; // Obliczenie części cosinus i sinus (Twiddle factors)
            for (k = j; k < n; k = k + n2) { // Pętla skacząca po głównym buforze
                t1 = c * rex[k + n1] - s * imx[k + n1]; // Część rzeczywista mnożenia
                t2 = c * imx[k + n1] + s * rex[k + n1]; // Część urojona mnożenia
                rex[k + n1] = rex[k] - t1; imx[k + n1] = imx[k] - t2; // Odejmij od dolnej połówki
                rex[k] = rex[k] + t1; imx[k] = imx[k] + t2; // Dodaj do górnej połówki
            }
        }
    }
}

// Funkcja trawiąca surowe dane z ADC
void Process_ADC_Data(uint16_t* source_buffer) {
    float imaginary_buffer[SAMPLES_PER_CH]; // Lokalny bufor na część urojoną dla FFT

    // 1. Demultipleksacja (Rozdzielenie przeplatanych danych z 4 kanałów na osobne tablice)
    for (int i = 0; i < SAMPLES_PER_CH; i++) {
        // CH0 (FFT): Odejmujemy 2048, aby usunąć składową DC (bias 1.65V) i mieć falę wokół 0
        ch0_fft_input[i] = (float)source_buffer[i * ADC_CHANNELS + 0] - 2048.0;
        imaginary_buffer[i] = 0.0; // Zerujemy część urojoną na start
        ch1_buffer[i]    = source_buffer[i * ADC_CHANNELS + 1]; // Zapisz surowe dane dla CH1
        ch2_buffer[i]    = source_buffer[i * ADC_CHANNELS + 2]; // Zapisz surowe dane dla CH2
        ch3_buffer[i]    = source_buffer[i * ADC_CHANNELS + 3]; // Zapisz surowe dane dla CH3
    }

    // 2. OBLICZENIA FFT (Kanał 0)
    FFT(ch0_fft_input, imaginary_buffer, SAMPLES_PER_CH); // Uruchomienie transformaty
    float max_magnitude = 0; // Zmienna na najwyższą znalezioną amplitudę
    int max_index = 0; // Indeks komórki, w której była najwyższa amplituda

    for (int i = 1; i < SAMPLES_PER_CH / 2; i++) { // Leci do połowy (odcięcie częstotliwości Nyquista). i=1 omija DC.
        // Twierdzenie Pitagorasa na liczbach zespolonych -> obliczenie Modułu (Amplitudy)
        float mag = sqrt(ch0_fft_input[i]*ch0_fft_input[i] + imaginary_buffer[i]*imaginary_buffer[i]);
        if (mag > max_magnitude) { // Jeśli ta amplituda jest największa...
            max_magnitude = mag; // ...zapamiętaj ją
            max_index = i; // ...oraz jej położenie w tablicy
        }
    }
    // Zapis do struktury globalnej. (max_index * 8000Hz) / 1024 próbek = Rzeczywista częstotliwość w Hz
    current_results.ch0_amplitude = (uint32_t)max_magnitude;
    current_results.ch0_dom_freq  = (max_index * 8000) / SAMPLES_PER_CH;

    // 3. OBLICZENIA STATYSTYCZNE (Kanały 1-3)
    uint32_t sum; // Zmienna pomocnicza do liczenia średniej
    uint16_t min_val = 4096, max_val = 0; // Zmienne do szukania maksimum i minimum (amplituda P-P)

    // CH1 (Sygnał zmienny AC)
    for(int i=0; i<SAMPLES_PER_CH; i++) {
        if(ch1_buffer[i] < min_val) min_val = ch1_buffer[i]; // Znajdź dolny punkt fali
        if(ch1_buffer[i] > max_val) max_val = ch1_buffer[i]; // Znajdź szczyt fali
    }
    current_results.ch1_avg = max_val - min_val; // Amplituda Peak-to-Peak (zapisana w polu "avg" dla spójności structa)
    BubbleSort(ch1_buffer, SAMPLES_PER_CH); // Sortuj bufor od najmniejszej do największej
    current_results.ch1_med = ch1_buffer[SAMPLES_PER_CH/2]; // Środkowy element to Mediana (tutaj: offset DC fali)

    // CH2 (Sygnał stały DC)
    sum = 0; for(int i=0; i<SAMPLES_PER_CH; i++) sum += ch2_buffer[i]; // Zsumuj wszystko
    current_results.ch2_avg = sum / SAMPLES_PER_CH; // Zwykła średnia arytmetyczna
    BubbleSort(ch2_buffer, SAMPLES_PER_CH); // Sortuj
    current_results.ch2_med = ch2_buffer[SAMPLES_PER_CH/2]; // Mediana (odporna na "szpilki" napięcia)

    // CH3 (Sygnał stały DC - analogicznie jak CH2)
    sum = 0; for(int i=0; i<SAMPLES_PER_CH; i++) sum += ch3_buffer[i];
    current_results.ch3_avg = sum / SAMPLES_PER_CH;
    BubbleSort(ch3_buffer, SAMPLES_PER_CH);
    current_results.ch3_med = ch3_buffer[SAMPLES_PER_CH/2];

    // 4. Historia
    // Jeśli minął określony czas (measure_interval_ms) od ostatniego logu...
    if (HAL_GetTick() - last_log_time >= measure_interval_ms) {
        last_log_time = HAL_GetTick(); // Zresetuj timer
        history_buffer[history_idx] = current_results; // Zapisz aktualne wyniki jako nową "klatkę" archiwum
        history_idx++; // Przejdź o jedno oczko dalej
        if (history_idx >= HISTORY_SIZE) history_idx = 0; // Jeśli historia jest pełna, nadpisuj najstarsze pomiary
    }
}

// Callback wywoływany przez sprzętowe DMA gdy przepisze połowę z 8192 bajtów bufora
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) { flag_half_transfer = 1; }
// Callback wywoływany przez DMA gdy dotrze do samego końca bufora (wtedy samo zaczyna znów od zera)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) { flag_full_transfer = 1; }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Rozruch głównej biblioteki obsługującej sprzęt (Hardware Abstraction Layer)

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Ustawia zegary procesora (np. PLL na 84MHz)

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();        // Inicjalizacja portów np. diody LED na płytce
  MX_DMA_Init();         // Włączenie zegarów dla sterowników DMA
  MX_USART2_UART_Init(); // Inicjalizacja sprzętowego UARTa do PC (Baudrate 115200)
  MX_ADC1_Init();        // Konfiguracja ADC (Skanowanie 4 kanałów)
  MX_TIM2_Init();        // Konfiguracja Timera 2 (Wyzwalacz ADC na 8000 Hz)
  /* USER CODE BEGIN 2 */

  // Uruchomienie UART w trybie przerwań. Czekamy na pierwszy (1) znak do komórki pod indeksem USART_RX_Busy.
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART_RxBuf[USART_RX_Busy], 1);

    // Start Timer (zacznij tykać 8000 razy na sekundę)
    HAL_TIM_Base_Start(&htim2);
    // Start ADC złączonego z DMA. Rozpocznij wrzucanie pomiarów prosto do pamięci RAM omijając procesor.
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, DMA_BUF_LEN);

    USART_fsend("SYSTEM READY\r\n"); // Pierwszy komunikat powitalny wysłany do PC
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) // Nieskończona główna pętla mikrokontrolera
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // --- 1. OBSŁUGA ADC / FFT ---
	      if (flag_half_transfer) { // Jeśli przerwanie zgłosiło, że pierwsza połowa danych jest gotowa
	          Process_ADC_Data(&adc_dma_buffer[0]); // Odpal DSP na danych zaczynając od indeksu 0
	          flag_half_transfer = 0; // Opuść flagę, czekaj na kolejną paczkę
	      }
	      if (flag_full_transfer) { // Jeśli przerwanie zgłosiło, że druga połowa danych jest gotowa
	          Process_ADC_Data(&adc_dma_buffer[DMA_BUF_LEN / 2]); // Odpal DSP na danych od połowy (indeks 4096)
	          flag_full_transfer = 0; // Opuść flagę
	      }

	      // --- 2. OBSŁUGA UART IT (Odbiór asynchroniczny, tzw. Bufor Kołowy) ---

          // Dopóki wskaźnik "Empty" nasłuchujący dogania głowę "Busy" wpisywaną przez przerwanie...
	      while (USART_RX_Empty != USART_RX_Busy) {

	                char rx_char = USART_RxBuf[USART_RX_Empty]; // Wyciągnij pojedynczy znak z bufora
	                USART_RX_Empty++; // Przesuń wzrok na następny znak
	                if (USART_RX_Empty >= USART_RXBUF_LEN) USART_RX_Empty = 0; // Zawinięcie bufora kołowego, jeśli dojechaliśmy do końca

	              // A. Synchronizacja (Twardy reset parsera przy znaku startu)
	              if (rx_char == '<') { // Oznacznik początku naszej tekstowej "koperty" ramki
	                  parser_state = FRAME_READ_PAYLOAD; // Zmień stan z uśpionego na "czytam zawartość"
	                  parser_idx = 0; // Zacznij budowanie nowego łańcucha od zera
	                  // Znak ucieczki zerujemy, a continue wymusza pobranie następnego znaku z ominięciem Switcha.
	                  continue;
	              }

	              static uint8_t escape_flag = 0; // Pamięć parsera mówiąca, że poprzedni znak wymusza dekodowanie

	              switch(parser_state) {
	                  case FRAME_WAIT_START: // Stan bezczynny
	                      break; // Śmieci lub ciąg znaków bez '<' lecą w próżnię

	                  case FRAME_READ_PAYLOAD: // Stan główny
	                      if (escape_flag == 1) { // Jeżeli poprzednio otrzymaliśmy ukośnik ('\')...
	                          escape_flag = 0; // Zużywamy flagę od razu

	                          // 1. DEKODOWANIE (Byte Stuffing - Tłumaczenie kodów z powrotem na oryginalne znaki)
	                          if (rx_char == '0') {
	                              raw_payload[parser_idx] = '\\'; // Kod \0 znaczy, że ktoś wysłał sam ukośnik
	                          }
	                          else if (rx_char == '1') {
	                              raw_payload[parser_idx] = '<'; // Kod \1 znaczy, że ktoś wysłał w danych znak <
	                          }
	                          else if (rx_char == '2') {
	                              raw_payload[parser_idx] = '>'; // Kod \2 znaczy, że ktoś wysłał w danych znak >
	                          }
	                          else {
	                              // BŁĄD PROTOKOŁU: 253 nieprawidłowe kombinacje! Odpalił ukośnik, a wysłał np. literę.
	                              // Odrzucamy całą, uszkodzononą ramkę i resetujemy maszynę
	                              parser_state = FRAME_WAIT_START;
	                              continue;
	                          }

	                          // 2. Ochrona pamięci (Warunek sprawdzany PO poprawnym zdekodowaniu)
	                          parser_idx++; // Inkrementuj pozycję tablicy
	                          if (parser_idx >= 300) { // Przepełenienie tablicy
	                              parser_state = FRAME_WAIT_START; // Zerwij ramkę, uniknij przepełnienia bufora (Buffer Overflow)
	                          }
	                      }
	                      else if (rx_char == '\\') {
	                          escape_flag = 1; // Zaczynamy sekwencję ucieczki (Nie dodajemy tego do bufora!)
	                      }
	                      else if (rx_char == '>') {
	                          // --- KONIEC RAMKI - ANALIZA CAŁKOWICIE BINARNA ---
                              // Skoro dotarliśmy tutaj, to mamy w raw_payload czyste, nierozbite binarne dane oraz tekstowe CRC

	                          // Min. długość: 4 bajty ID (PCST) + 4 bajty CRC = 8 bajtów niezbędnego minimum
	                          if (parser_idx >= 8) {
	                              int crc_start_idx = parser_idx - 4; // Punkt, w którym urywa się payload, a zaczyna 4-znakowe sumaryczne CRC

	                              // 3. Ręczna konwersja CRC bez użycia strtol() (Szybkie i bezpieczne rzutowanie z HEX ASCII na Int)
	                              uint16_t recv_crc = 0;
	                              for (int i = 0; i < 4; i++) {
	                                  char c = raw_payload[crc_start_idx + i]; // Pobierz jeden z czterech znaków CRC
	                                  recv_crc <<= 4; // Przesuwamy wynik o 4 bity w lewo (robimy miejsce dla nowej cyfry HEX)
	                                  if (c >= '0' && c <= '9')      recv_crc |= (c - '0'); // Jeśli to 0-9, wrzuć w najmłodsze bity
	                                  else if (c >= 'A' && c <= 'F') recv_crc |= (c - 'A' + 10); // Jeśli to A-F, przeskaluj i wrzuć
	                              }

	                              // Własne CRC obliczone przez STM (od indeksu 0 do miejsca, gdzie zaczyna się przysłane CRC)
	                              uint16_t calc_crc = Calculate_CRC(raw_payload, crc_start_idx);

	                              if (calc_crc == recv_crc) { // Jeżeli CRC sprzętu i CRC ramki są identyczne...

	                                  // 4. Rozpoznawanie ID i Komend BEZ użycia funkcji C-Stringowych (ręcznie na bajtach, ignorując zera)
	                                  if (raw_payload[0] == 'P' && raw_payload[1] == 'C' &&  // ID Nadawcy = PC
	                                      raw_payload[2] == 'S' && raw_payload[3] == 'T')    // ID Odbiorcy = ST
	                                  {
	                                      stat_rx_count++; // Zwiększ statystyki dobrych pakietów

	                                      uint8_t* data_ptr = (uint8_t*)&raw_payload[4]; // Wskaźnik na początek właściwego ładunku (Danych)
	                                      int data_len = crc_start_idx - 4; // Wyliczona długość właściwego ładunku

	                                      // Weryfikacja komendy "GET CH0" (sprawdzamy idealne dopasowanie długości i liter)
	                                      if (data_len == 7 &&
	                                          data_ptr[0] == 'G' && data_ptr[1] == 'E' && data_ptr[2] == 'T' &&
	                                          data_ptr[3] == ' ' && data_ptr[4] == 'C' && data_ptr[5] == 'H' && data_ptr[6] == '0')
	                                      {
	                                          // Budowa binarnej odpowiedzi dla CH0 zgodnie z tabelą w dokumentacji
	                                          uint8_t bin_payload[13];
	                                          bin_payload[0]='C'; bin_payload[1]='H'; bin_payload[2]='0'; bin_payload[3]='F'; // "CH0F"

                                              // Przekształcanie 32-bitowej liczby do 4 bajtów - Protokół Network Big Endian
	                                          bin_payload[4] = (current_results.ch0_dom_freq >> 24) & 0xFF; // Najbardziej znaczący bajt
	                                          bin_payload[5] = (current_results.ch0_dom_freq >> 16) & 0xFF; // Drugi bajt
	                                          bin_payload[6] = (current_results.ch0_dom_freq >> 8)  & 0xFF; // Trzeci bajt
	                                          bin_payload[7] =  current_results.ch0_dom_freq        & 0xFF; // Najmniej znaczący bajt

                                              bin_payload[8] = 'A'; // "A" - Amplituda

                                              // Kolejne rozbijanie na części Big Endian
	                                          bin_payload[9]  = (current_results.ch0_amplitude >> 24) & 0xFF;
	                                          bin_payload[10] = (current_results.ch0_amplitude >> 16) & 0xFF;
	                                          bin_payload[11] = (current_results.ch0_amplitude >> 8)  & 0xFF;
	                                          bin_payload[12] =  current_results.ch0_amplitude        & 0xFF;

                                              Send_Binary(bin_payload, 13); // Odpal zabezpieczoną funkcję nadającą
	                                      }
	                                      // Weryfikacja komendy "GET CH1"
	                                      else if (data_len == 7 &&
	                                               data_ptr[0] == 'G' && data_ptr[1] == 'E' && data_ptr[2] == 'T' &&
	                                               data_ptr[3] == ' ' && data_ptr[4] == 'C' && data_ptr[5] == 'H' && data_ptr[6] == '1')
	                                      {
	                                          // Budowa binarnej odpowiedzi dla CH1
	                                          uint8_t bin_payload[13];
	                                          bin_payload[0]='C'; bin_payload[1]='H'; bin_payload[2]='1'; // "CH1"
	                                          bin_payload[3]='A'; bin_payload[4]='V'; bin_payload[5]='G'; // "AVG"

                                              // AVG to liczba 16-bitowa, rozbijamy na 2 bajty (Big Endian)
	                                          bin_payload[6] = (current_results.ch1_avg >> 8) & 0xFF; // Górny bajt (MSB)
	                                          bin_payload[7] =  current_results.ch1_avg       & 0xFF; // Dolny bajt (LSB)

	                                          bin_payload[8]='M'; bin_payload[9]='E'; bin_payload[10]='D'; // "MED"

                                              // MED to liczba 16-bitowa
	                                          bin_payload[11] = (current_results.ch1_med >> 8) & 0xFF; // MSB
	                                          bin_payload[12] =  current_results.ch1_med       & 0xFF; // LSB

	                                          Send_Binary(bin_payload, 13); // Odpal funkcję nadającą
	                                      }
	                                  }
	                              }
	                          }
	                          // Koniec bloku analizy, po przetworzeniu udanej (lub błędnej z powodu złego CRC) ramki resetujemy maszynę
	                          parser_state = FRAME_WAIT_START; // Czekamy na nowe "<"
	                          escape_flag = 0; // Ubezpieczenie - kasowanie flag ucieczki
	                      }
	                      else {
	                          // Tryb normalnego zapisu "niegroźnych" znaków
	                          raw_payload[parser_idx] = rx_char; // Przypisz znak do tablicy

	                          // 2. Warunek sprawdzany PO zmianie parametru, by chronić wskaźnik
	                          parser_idx++;
	                          if (parser_idx >= 300) {
	                              parser_state = FRAME_WAIT_START; // Zerwij ramkę, unikaj Buffer Overflow
	                          }
	                      }
	                      break;

	                  default:
	                      parser_state = FRAME_WAIT_START; // Gdyby maszyna weszła w zły stan, rzuć w domyślny
	                      break;
	              }

	          }
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration (Wygenerowane przez CubeMX - inicjalizacja zegarów np. PLL)
  * @retval None
  */
void SystemClock_Config(void)
{
    // CUBEMX AUTO-GENERATE ...
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function (Konfiguruje przetwornik skanujący kanały)
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    // CUBEMX AUTO-GENERATE ...
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B; // Przetwornik 12-bitowy (0 - 4095)
  hadc1.Init.ScanConvMode = ENABLE; // Wymusza odczyt wielu kanałów naraz podczas jednego pomiaru
  hadc1.Init.ContinuousConvMode = DISABLE; // Skanowanie wyzwalane jest sprzętowo, nie pętlą stałą
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; // Pomiar startuje na sygnał zbocza narastającego z Timera 2
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4; // Będziemy skanować 4 kanały podczas jednego cyklu
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Ranga (kolejność) pierwsza
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Ranga druga
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Ranga trzecia
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Ranga czwarta
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM2 Initialization Function (Wyzwala ADC np. 8000 razy na sekundę)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    // CUBEMX AUTO-GENERATE ...
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0; // Brak preskalera (szybkie uderzenia)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295; // Wartość ustalająca częstotliwość f_s (np. dobita w CubeMX na 8kHz)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  // Timer wypluwa zdarzenie UPDATE (TRGO) które jest uderzeniem pobudzającym ADC
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USART2 Initialization Function (Komunikacja szeregowa)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    // CUBEMX AUTO-GENERATE ...
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Szybkość transmisji 115200 bitów/s
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX; // Nadawanie i Odbieranie włączone
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE(); // Włączenie zegara kontrolera DMA

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0); // Ustalenie priorytetu wywołań (0 = najwyższy)
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn); // Aktywacja pinu obsługi przerwań sprzętowych
}

/**
  * @brief GPIO Initialization Function (Konfiguracja wejść/wyjść na płycie)
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Port m.in. przycisku uzytkownika B1
  __HAL_RCC_GPIOH_CLK_ENABLE(); // Zegar systemowy
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Port np. Dioda LD2, piny ADC
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Inne komponenty

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Ustawia diodę (Pin A5 na płytce) domyślnie na wyłączoną

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Przerwanie przy puszczeniu guzika User Button
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Tryb wyjściowy Push-Pull dla diody LED
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// obowiazkowe
/* USER CODE BEGIN 4 */

// Callback wykonywany sprzętowo po otrzymaniu DOKŁADNIE jednego znaku (IT RxCplt) przez UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) // Upewniamy się, że to przerwanie wywołał właściwy UART2
    {
        // 1. Opcjonalne mrugnięcie (np. dla debugowania na płycie)
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        // 2. Przesuwamy głowę (wskaźnik wpisania najnowszego znaku) o jedno miejsce dalej
        USART_RX_Busy++;

        // Jeśli dotarliśmy do końca tablicy (indeks 256) -> zawijamy na 0. (Struktura Bufora Kołowego)
        if(USART_RX_Busy >= USART_RXBUF_LEN) {
            USART_RX_Busy = 0;
        }

        // 3. Włączamy ponownie czujkę sprzętową na otrzymanie kolejnego pojedyńczego (1) znaku do zaktualizowanego miejsca w tablicy
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART_RxBuf[USART_RX_Busy], 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq(); // Jeśli mikrokontroler padnie krytycznie, blokuje wszystkie przerwania
  while (1) // Wrzuca program w zamrożenie dla bezpieczeństwa
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
