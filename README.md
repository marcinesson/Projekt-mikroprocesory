# Rejestrator 4CH ADC + FFT (STM32F401RE)

To jest repozytorium mojego projektu rejestratora sygnałów analogowych. Całość śmiga na mikrokontrolerze STM32F401RE (Nucleo). Projekt zbiera dane z 4 kanałów ADC, mieli je matematycznie (m m.in. FFT, mediana, średnia) i wysyła gotowe wyniki do PC po UART używając własnego, zabezpieczonego protokołu binarnego.

## Co tu właściwie działa?

*   **Pomiary 8 kHz bez zacinania:** ADC jest wyzwalane sprzętowo przez Timer 2, a dane lecą bezpośrednio do pamięci przez DMA w trybie kołowym (Ping-Pong z przerwaniami Half/Full Transfer). Procesor w ogóle nie traci na to czasu.
*   **Analiza widmowa (FFT):** Na Kanale 0 (PA0) wlatuje sygnał zmienny. Algorytm usuwa składową stałą, robi 1024-punktowe FFT i wypluwa częstotliwość dominującą oraz jej amplitudę.
*   **Statystyki AC/DC:** 
    *   Kanał 1 liczy amplitudę Peak-to-Peak (zwykłe Max - Min) i medianę dla sygnałów zmiennych.
    *   Kanały 2 i 3 liczą średnią i medianę dla sygnałów stałych (do mediany użyłem Bubble Sorta, żeby odfiltrować "szpilki" i śmieci w sygnale).
*   **Własny parser UART:** Komunikacja z PC (115200 baud) działa w 100% asynchronicznie na przerwaniach i buforze kołowym. Nic się nie blokuje.

## Pinout (Gdzie co podpiąć)

| Pin na Nucleo | Co to robi |
| :--- | :--- |
| `PA0` (A0) | Wejście ADC CH0 -> idzie prosto do FFT |
| `PA1` (A1) | Wejście ADC CH1 -> parametry fali AC (Vpp, Mediana) |
| `PA4` (A2) | Wejście ADC CH2 -> sygnał DC (np. z fotorezystora) |
| `PA8` (D7) | Wejście ADC CH3 -> sygnał DC |
| `PA2` / `PA3` | UART TX/RX -> komunikacja z PC (idzie przez USB na płytce) |
| `PA5` | Wbudowana dioda LED |

## Protokół komunikacyjny

Gadam z płytką wysyłając i odbierając ramki binarne zabezpieczone kodowaniem. 

**Kształt ramki:**
`<` `[ID Nadawcy]` `[ID Odbiorcy]` `[Surowe Dane]` `[CRC-16]` `>`

Żeby wysyłanie surowych bajtów (np. wyników z FFT) nie zepsuło parsowania (bo np. bajt danych ma taką samą wartość jak znak `<`), zastosowałem **Byte Stuffing**:
*   Znak `\` leci jako `\0`
*   Znak `<` leci jako `\1`
*   Znak `>` leci jako `\2`

Suma kontrolna to standardowe CRC-16 (XOR z `0x1021`), wysyłane jako 4 znaki HEX (np. `A1B2`) tuż przed zamknięciem ramki.

### Zaimplementowane komendy

Jak wyślesz z PC poprawną ramkę z nagłówkiem `PCST` (Nadawca PC, Odbiorca ST), STM32 odpowie binarnie (w formacie Big Endian). 

| Komenda (w polu danych) | Odpowiedź płytki |
| :--- | :--- |
| `GET CH0` | Zwraca wynik FFT z CH0 (Częstotliwość i Amplitudę) |
| `GET CH1` | Zwraca statystyki z CH1 (Amplituda P-P i Mediana) |
| `SET INT 1000` | Ustawia co ile milisekund (tu: 1000) robić zrzut do historii |
| `GET ARCH 0` | Wyciąga paczkę danych z bufora historii pod konkretnym indeksem |

## Jak to odpalić?

1. Ściągnij repo i otwórz folder projektu w **STM32CubeIDE**.
2. Zbuduj wsad i wgraj na STM32F401RE.
3. Odpal RealTerma, Putty albo własną apkę na PC (ustawienia portu: `115200, 8N1`).
