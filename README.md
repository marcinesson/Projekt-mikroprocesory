# Rejestrator Sygnałów 4CH ADC + Analiza Widmowa (FFT) na STM32

Zaawansowany system wbudowany oparty na mikrokontrolerze **STM32F401RE** (rdzeń Cortex-M4), realizujący precyzyjną akwizycję danych analogowych oraz ich cyfrowe przetwarzanie sygnałów (DSP) w czasie rzeczywistym.

## Główne Funkcje

*   **Sprzętowa Akwizycja Danych:** 4 kanały ADC próbkowane z częstotliwością **8 kHz**, wyzwalane w 100% sprzętowo przez Timer (TIM2), co eliminuje *jitter* (opóźnienia programowe).
*   **Bezpośredni Dostęp do Pamięci (DMA):** Wykorzystanie trybu Circular Mode z przerwaniami Half-Transfer oraz Transfer-Complete (tzw. mechanizm **Ping-Pong**). Gwarantuje to brak utraty próbek podczas analizy danych przez rdzeń.
*   **Cyfrowe Przetwarzanie Sygnałów (DSP):**
    *   **Kanał 0:** Analiza widmowa za pomocą algorytmu Szybkiej Transformaty Fouriera (**FFT**). System usuwa składową stałą, wylicza moduł liczb zespolonych i zwraca częstotliwość dominującą (w Hz) oraz jej amplitudę.
    *   **Kanał 1:** Pomiar parametrów sygnału zmiennego (AC) - amplituda międzyszczytowa (**Peak-to-Peak**) oraz wyznaczanie punktu pracy (Offset/Mediana).
    *   **Kanały 2 i 3:** Odczyty sygnałów stałych (DC) z nieliniową filtracją zakłóceń impulsowych z użyciem algorytmu **sortowania bąbelkowego (Bubble Sort)** do wyznaczania mediany.
*   **Niezawodna Komunikacja:** Interfejs UART (115200 baud) zbudowany na bazie asynchronicznych przerwań (IT) i **bufora kołowego (Ring Buffer)**, obsługujący autorski, bezpieczny protokół binarny.

## Wyprowadzenia (Pinout)

| Peryferium | Pin STM32 | Funkcja w systemie |
| :--- | :--- | :--- |
| **ADC1_IN0** | `PA0` | Kanał 0: Sygnał wejściowy do analizy FFT (0-4000 Hz) |
| **ADC1_IN1** | `PA1` | Kanał 1: Pomiary statystyczne sygnału zmiennego (AC) |
| **ADC1_IN4** | `PA4` | Kanał 2: Pomiary sygnału stałego (DC) - np. czujniki |
| **ADC1_IN8** | `PA8` | Kanał 3: Pomiary sygnału stałego (DC) - np. czujniki |
| **USART2_TX** | `PA2` | Transmisja danych do komputera PC |
| **USART2_RX** | `PA3` | Odbieranie komend od komputera PC |
| **LED (LD2)**| `PA5` | Dioda sygnalizacyjna na płytce Nucleo |

## Protokół Komunikacyjny

Komunikacja między komputerem (PC) a mikrokontrolerem (ST) odbywa się za pomocą asynchronicznych ramek z mechanizmem **Byte Stuffing** (kodowanie znaków ucieczki) i sumą kontrolną **CRC-16**.

**Struktura Ramki:**
`[<] [ID_NADAWCY] [ID_ODBIORCY] [DANE] [CRC16_HEX] [>]`

*   **Początek/Koniec:** Ramka zawsze zaczyna się od znaku `<` (0x3C) i kończy znakiem `>` (0x3E).
*   **Adresowanie:** Wymagany nagłówek dla komend to `PCST` (Nadawca: PC, Odbiorca: ST).
*   **Zabezpieczenie Danych (Byte Stuffing):** Jeśli w binarnej sekcji DANE wystąpią znaki kolidujące ze strukturą ramki, są one maskowane:
    *   Znak `\` (0x5C) wysyłany jest jako `\0`
    *   Znak `<` (0x3C) wysyłany jest jako `\1`
    *   Znak `>` (0x3E) wysyłany jest jako `\2`
*   **Suma kontrolna:** 16-bitowa suma kontrolna obliczana na podstawie ładunku i ID, rzutowana na 4 znaki w formacie szesnastkowym (np. `1A2F`), wykorzystująca operację XOR ze stałą `0x1021`.

### 📋 Dostępne Komendy
| Rozkaz | Długość (b) | Opis |
| :--- | :--- | :--- |
| `GET CH0` | 7 | Zwraca binarnie (Big Endian) wyniki FFT z Kanału 0: Częstotliwość (4b) i Amplitudę (4b). |
| `GET CH1` | 7 | Zwraca binarnie statystyki z Kanału 1: Amplitudę P-P (2b) i Medianę (2b). |
| `SET INT x` | >8 | Ustawia interwał zapisu do historii na wartość `x` w milisekundach. |
| `GET ARCH i`| >9 | Odczytuje zapisane parametry z archiwum z bufora cyklicznego pod indeksem `i`. |

## Wymagania i Kompilacja

Projekt został stworzony i skonfigurowany w środowisku **STM32CubeIDE**. Do poprawnego zbudowania kodu wymagane są biblioteki z pakietu **STM32Cube MCU Package for STM32F4** (warstwa HAL).

1. Sklonuj to repozytorium do swojego obszaru roboczego (Workspace).
2. Otwórz projekt w STM32CubeIDE.
3. Zbuduj projekt (Project -> Build Project).
4. Wgraj wsad (.elf/.bin) na mikrokontroler z użyciem wbudowanego programatora ST-LINK.
5. Uruchom dowolny terminal szeregowy na PC z parametrami: **115200 baud, 8 bitów danych, brak parzystości, 1 bit stopu (8-N-1)**.
