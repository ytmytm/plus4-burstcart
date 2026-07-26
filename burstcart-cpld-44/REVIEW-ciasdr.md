# Dlaczego burstcart-cpld-44 gubi bajty i przesuwa bity

Analiza z 2026-07-26. Hipoteza z `PROMPT.txt` była trafna w jednym punkcie
(problem jest w firmware CPLD), ale **nietrafna w założeniu, że układ KiCad
odpowiada temu, co robi wersja VIA** — płytki różnią się elektrycznie i to jest
sedno sprawy.

---

## 1. Fakty ustalone z dokumentacji i ROMu (nie domysły)

### Co robi stacja 1581

Z ROMu 1581 (`1581-rom/AAY1581.TXT`), inicjacja w `$AF3A`:

```
AF3A: A9 00     LDA #$00         Timer A fuer FSM auf $0006 stellen
AF3C: 8D 05 40  STA $4005
AF3F: A9 06     LDA #$06
AF41: 8D 04 40  STA $4004
```

Timer A = 6 przy 2 MHz → underflow co 3,5 µs, CNT przełącza się na każdym
underflow, więc:

| parametr | wartość |
|---|---|
| okres bitu | **7 µs** |
| faza CNT wysoka / niska | 3,5 µs / 3,5 µs (50 % duty) |
| bajt (8 bitów) | 56 µs |
| turnaround stacji (ack → pierwszy bit) | ~27 µs |
| kolejność bitów | MSB first |

Nadajnik to sprzętowy SDR w CIA, nigdzie nie ma bit-bangingu SRQ. Blok
fastload (`$B977`) wysyła bajty przez `$01FC → $BA40`, a handshake to
**porównanie poziomu**, nie zbocze: `EOR $76 / AND #$04` — więc ack hosta
może przyjść w dowolnym momencie i nie zginie.

### Które zbocze i gdzie są dane

Datasheet 6526 (`docs/mos_6526_cia_preliminary_nov_1981.pdf`, s. 7), dosłownie:

> „In input mode, data on the SP pin is shifted into the shift register **on the
> rising edge** of the signal applied to the CNT pin."
>
> „Data shifted out becomes valid **on the falling edge of CNT** and remains
> valid until the next falling edge."

To samo u Ojali (`docs/c=hacking19.txt`): *„A bit is valid between clock falling
edges, the data is sampled during rising edges."*

Czyli: dane zmieniają się na zboczu **opadającym**, a zbocze **narastające**
leży dokładnie w środku bitu, z 3,5 µs marginesu z każdej strony. `posedge CNT`
w `ciasdr.v` jest więc semantycznie **poprawne** — i nie ma żadnej inwersji na
ścieżce (SP/CNT są open-drain i idą wprost na magistralę IEC, potwierdzone i w
datasheecie, i w schematach obu płytek). Polaryzacja **nie jest** problemem.

### Czym różni się pracująca płytka VIA — to jest klucz

Ze schematów (`*.kicad_sch` + netlisty z `.kicad_pcb`):

| ścieżka | burstcart-via (działa) | burstcart-cpld-44 (nie działa) |
|---|---|---|
| SRQ → zegar SR | `IEC_SRQ` → **74LS74 (U2A), D=SRQ, CLK=/VIACLK (Phi2)** → 74LS126 (U5B) → CB1 (pin 18) | `IEC_SRQ` → **wprost na pin 43 CPLD** |
| DATA → dane SR | `IEC_DATA` → 74LS126 (U5C) → CB2 (pin 19) | `IEC_DATA` → wprost na pin 38 |
| pull-up | R3/R2 3k3 → VCC | R3/R2 3k3 → VCC (identyczne) |
| inwersje | 0 | 0 |
| złącza IEC | jedno (J1) | **dwa (J1+J2, przelotka)** |

Na płytce VIA jest **przerzutnik 74LS74, który resynchronizuje SRQ do zegara
systemowego, zanim trafi na CB1**. To nie przypadek — `docs/via.txt` mówi
wprost:

> „To remedy it, put the external clock signal into the D input of a 74HC74
> flip-flop, run the flip-flop's Q output to the 6522's CB1 pin, and clock the
> flip-flop with phase 0 or phase 2."

A sam 6522 dodatkowo próbkuje synchronicznie. Datasheet W65C22 §2.12.4 (tryb
011, ten używany w `burst-via.asm`: `lda #%00001100 / sta via_acr`):

> „Note that data is shifted **during the first PHI2 clock cycle following the
> positive going edge of the CB1 shift pulse**. For this reason, data must be
> held stable during the first full cycle following CB1 going high."

I druga, równie ważne zdanie z tego samego akapitu:

> „**Reading or writing the SR resets IFR2 and initializes the counter to count
> another eight pulses.**"

---

## 2. Trzy usterki w `ciasdr.v`

### Usterka 1 (główna): surowa linia IEC SRQ użyta jako zegar sprzętowy

```verilog
always @(posedge CNT or negedge sp_in_reset_n) begin
    ...
    shift_in <= {shift_in[6:0], SP};
```

`CNT` to pin 43, zmapowany przez fitter na **globalną sieć zegarową GCK1**
(`cia.rpt:29`). Czyli rejestr przesuwny jest taktowany bezpośrednio linią
magistrali IEC, która:

* jest open-drain z pull-upem 3k3 do +5 V, obciążonym kablem i **dwoma**
  złączami IEC → zbocze narastające to wolna rampa RC rzędu 0,5–1 µs;
* wchodzi na wejście zegarowe **XC9572XL, które nie ma histerezy** (a to część
  3,3 V, więc próg jest jeszcze niżej względem 5 V rampy).

Każde zakłócenie na tej rampie przekracza próg po raz drugi i **taktuje rejestr
przesuwny ponownie**. Jeden dodatkowy impuls = jeden fantomowy bit = bajt
obrócony, flaga „bajt gotowy" wypada o zbocze za wcześnie, i od tego momentu
cały strumień jest przesunięty. Dokładnie objaw z `PROMPT.txt`.

Żaden z prawdziwych układów tego nie robi: 6522 próbkuje w rytmie PHI2, a
płytka VIA dodaje jeszcze 74LS74. Prawdziwe CIA to NMOS na tej właśnie
magistrali. CPLD z propagacją ~5 ns na rampie 1 µs — nie.

### Usterka 2: licznik bitów nigdy się nie resynchronizuje

Stary licznik `shift_in_counter` był zerowany **tylko** przez RESET albo przez
przełączenie portu na wyjście (`sp_in_reset_n = RESET_n && !sp_output`). W
trakcie całej transmisji software nie pisze do `cpldbase+1`, więc licznik biegnie
swobodnie modulo 8. Pojedyncze zgubione albo dodatkowe zbocze przesuwa **na
zawsze** wszystkie kolejne bajty — nie ma bitu startu, jedynym framingiem jest
ten licznik.

Że to realne ryzyko, wie sam ROM 1581 (`AAY1581.TXT`, blok patchy
„FSM: Schieberegister initialisieren"):

> „Wenn erst der Computer und dann die 1581 eingeschaltet wird, kann es
> vorkommen, dass im Computer irrtuemlich ein paar Bits ‚empfangen' werden.
> **Bei einer Datenuebertragung wuerde der Computer anschliessend alle Bits
> verschoben empfangen.**"

Dlatego stacja przed **każdą** transmisją przełącza kierunek SDR 0→1→0
(`$DBC7`/`$DBE0`), żeby wyzerować swój własny licznik. Dodatkowo stacja wysyła
przed transferem bajt-budzik `$00` (`$AC9D`) i owija bajty statusu w kolejne
przełączenia kierunku — na drucie jest więc więcej ruchu, niż widzi pętla
loadera.

6522 jest odporny, bo zeruje licznik przy każdym czytaniu SR — i to jest powód,
dla którego wersja VIA sama się leczy.

### Usterka 3: flaga statusu próbkowała magistralę na złym zboczu

```verilog
always @(posedge E_CLK or negedge RESET_n) begin        // <-- posedge
    ...
    else if (seladdr && RW && A[0] == REG_SDR)
        shift_complete_latched <= 1'b0;
```

Na Plus/4 przy narastającym zboczu PHI0 magistrala adresowa **jeszcze nie
trzyma adresu CPU**. Wszystkie zapisy rejestrów w tym samym pliku używają
`negedge E_CLK` — i one działają (potwierdzone: rejestry dają się zapisywać, a
Twoje notatki w `docs/burstc64.txt` zapisują regułę Plus/4:
*„drivedatain: adres+<WRITE>+negedge pla[6] (zapis do rejestru)"*). Czyli
„skasuj flagę przy odczycie rejestru danych" wypadało w złym momencie albo
wcale.

To jest **regresja wprowadzona później**: w raporcie fittera z 2026-03-07
(`cia.rpt`) ten sam przerzutnik jest jeszcze na `NOT E_CLK`, czyli na zboczu
opadającym. Ta sama wersja miała też dwustopniowy synchronizator
`req_sync1`/`req_sync2` na przejściu domen, którego w obecnym `ciasdr.v` już nie
ma.

### Uwaga na marginesie: plik JED nie odpowiada żadnemu źródłu

```
cia.jed             Date Extracted: Thu May 29 23:38:38 2025
cia — kopia.jed     Date Extracted: Thu May 29 00:23:47 2025
cia.jed-20250608    Date Extracted: Thu May 29 23:38:38 2025
```

Wszystkie trzy pliki programujące są z **28–29 maja 2025**, mimo że mtime
`cia.jed` to 2026-03-07. Raport fittera i `cia.vm6` są z 2026-03-07 12:55 i
zawierają `req_sync1`/`req_sync2`, których nie ma w żadnym `.v` na dysku, a samo
`ciasdr.v` było edytowane jeszcze później (22:16). Czyli **układ na płytce
zawiera logikę z 29 maja 2025, nie tę, którą debugujemy.** Warto to
uporządkować, zanim wyciągnie się wnioski z kolejnego testu na sprzęcie.

---

## 3. Symulacja

`sim/burstsim.py` i `sim/verify_new.py` modelują nadajnik 1581 (7 µs/bit, dane
na zboczu opadającym, handshake per bajt), pętlę `GetByte`/`GetAndStore` z
`parobek/burst-cpld.asm` z prawdziwymi liczbami cykli 6502, oraz trzy odbiorniki:
stary `ciasdr.v`, nowy `ciasdr.v` i układ z płytki VIA (74LS74 + 6522 tryb 011).

`verify_new.py` to model **cyklicznie dokładny** — po przepisaniu całego
projektu na jedną domenę (`negedge E_CLK`) jedna ewaluacja na okres PHI0
odtwarza RTL dokładnie.

```
PHI0 = 0.887 MHz (1128 ns), 1581 bit period 7.0 us (3.10 PHI0 cycles per CNT phase)

1. wake-up handshake (CRA=$40, SDR=$08, poll bit 3): flag raised after 30 PHI0
   cycles, 8 CNT pulses generated

2. clean 1581 stream
   new ciasdr.v          recv=24/24 correct=24 wrong_at=[]                 -> PASS
   old ciasdr.v          recv=24/24 correct=24 wrong_at=[]                 -> PASS

3. 200 ns threshold re-crossing on every SRQ rising edge
   new ciasdr.v          recv=24/24 correct=24 wrong_at=[]                 -> PASS
   old ciasdr.v          recv=24/24 correct= 0 wrong_at=[0,1,2,3,4,5]      -> FAIL

4. one stray SRQ pulse after byte 5 (recovery behaviour)
   new ciasdr.v          recv=24/24 correct=24 wrong_at=[]                 -> PASS
   old ciasdr.v          recv=24/24 correct= 6 wrong_at=[6,7,8,9,10,11]    -> FAIL
```

Test 3 to usterka 1: **jedno przekroczenie progu o szerokości 200 ns na każdym
zboczu narastającym psuje wszystkie bajty**, a dla wersji synchronicznej jest
całkowicie niewidoczne. Test 4 to usterka 2: jeden zabłąkany impuls psuje stary
projekt **od tego bajtu do końca**, nowy odzyskuje synchronizację natychmiast.

Zamiatanie szerokości zakłócenia (`burstsim.py`, sekcja sweep) pokazuje granice:

```
  glitch      PHI0 |   cpld_old          cpld_new           via_ref
    50ns  0.887MHz |       PASS              PASS              PASS
   100ns  0.887MHz |       FAIL              PASS              PASS
   200ns  0.887MHz |       FAIL              PASS              PASS
   400ns  0.887MHz |       FAIL              FAIL              FAIL
```

Nowa wersja ma **dokładnie taką samą odporność jak płytka, która działa** —
i to był cel, nie „lepiej niż VIA". Przy zakłóceniach ≳1/3 okresu PHI0 obie
padają jednakowo; to obszar, w którym pomoże już tylko sprzęt.

Osobny test w `burstsim.py` (przypadek E) pokazuje usterkę 3 w izolacji: przy
czystym sygnale, ale próbkowaniu magistrali w złym momencie, stary projekt
odbiera **0 bajtów** — czyli zawisa, tak jak w notatce z 20250530.

---

## 4. Co zostało zmienione w `ciasdr.v`

Cały projekt jest teraz w **jednej domenie zegarowej: `negedge E_CLK` (PHI0)**.

1. `CNT` i `SP` są synchronizowane do PHI0 (`cnt_s1/s2/s3`, `sp_s1/s2`) i zbocze
   narastające jest detekowane w tej domenie. `sp_s2` jest próbkowane w tym samym
   cyklu, w którym `cnt_s2` pierwszy raz zobaczył CNT wysoko — czyli w granicach
   1,2 µs od zbocza, przy 3,5 µs ważności danych z każdej strony.
   Budżet: faza CNT = 3,5 µs = **3,10 cyklu PHI0** przy 0,886 MHz i 6,2 przy
   1,77 MHz — sprawdzone symulacją na obu prędkościach (ekran włączony/wyłączony).
2. `shift_in_counter` jest zerowany przy **każdym dostępie do rejestru danych**
   (`acc_sdr`), czyli framing resynchronizuje się raz na bajt — zachowanie 6522.
   Nadal zerowany też przy przełączeniu na wyjście (zachowanie prawdziwego CIA,
   na którym opiera się ROM 1581).
3. `shift_complete_latched` jest na `negedge E_CLK`, tak samo jak zapisy
   rejestrów, które działają.
4. Efekt uboczny: `CNT` przestaje być zegarem, więc zwalnia globalną sieć GCK1.
   Ścieżka nadawania straciła niepotrzebny bufor drugiego bajtu
   (`sdr_out_new_data`) — software wysyła tylko jeden bajt-budzik, którego
   zawartość nie ma znaczenia. Liczba przerzutników bez zmian (47), liczba
   termów iloczynowych spada, więc powinno zmieścić się swobodniej niż
   poprzednie 66/72 makrokomórek.

Poprzednia wersja jest w `ciasdr-orig-backup.v`.

## 5. Czego nie sprawdziłem / co zrobić dalej

* **Nie przesyntetyzowałem projektu** — na tej maszynie nie ma ISE. Trzeba
  przepuścić przez XST + cpldfit i sprawdzić, czy mieści się w 72 makrokomórkach
  (przewiduję tak, ale FB1/FB2/FB4 były wyczerpane przy 18/18).
* **Zaprogramować świeży JED** — obecny jest z maja 2025 i nie odpowiada
  żadnemu źródłu w projekcie.
* Jeśli po tym nadal będą błędy: dodać na płytce to, co ma wersja VIA, czyli
  przerzutnik resynchronizujący SRQ (74HC74 taktowany PHI0) albo bufor z
  histerezą (74HCT14 / 74LVC1G17) na SRQ i DATA przed CPLD. Firmware tego już
  nie potrzebuje, ale przy zakłóceniach >400 ns nic innego nie pomoże.
* Rozważyć dołożenie `!MUX` do dekodera zapisu i kasowania flagi. **Nie zrobiłem
  tego celowo** — zapisy rejestrów bez `!MUX` demonstracyjnie działają, więc
  dokładanie tam warunku to ryzyko regresji. Warto spróbować tylko jeśli po
  powyższych zmianach zostaną sporadyczne błędy.

---

## 6. Zgodność z `parobek/burst-cpld.asm` — kod 6502 bez zmian

Mapa rejestrów, semantyka flagi i bitu kierunku są nietknięte:

| adres | R | W |
|---|---|---|
| `$FD90` (`cpldbase`) | dane odebrane (`sdr_in`); kasuje flagę **i resynchronizuje licznik bitów** | dane do wysłania, start nadawania |
| `$FD91` (`cpldbase+1`) | bit 6 = kierunek, bit 3 = shift complete; **nie** kasuje flagi | bit 6 = kierunek; kasuje flagę |

Jedyna zmiana widoczna dla software'u to zerowanie licznika bitów przy dostępie
do `$FD90` — a to jest dokładnie to, co loader i tak robi raz na bajt
(`ldy cpldbase` w `GetByte`), więc wychodzi „za darmo". Dodatkowo `lda cpldbase`
przed `jsr ToggleClk` na starcie transferu ustawia framing tuż przed pierwszym
bajtem.

`sim/test_init.py` odtwarza dosłownie sekwencję `InitBurst` + detekcję CPLD z
`burst-cpld.asm` (z prawdziwymi liczbami cykli) na modelu nowego RTL:

```
InitBurst:
   [ok ] sta $FD91,#$00  -> sp_output = 0 (serial IN)
   [ok ]                 -> flag cleared
LoadBurst, CPLD presence detection:
   [ok ] lda $FD91 / cmp $FD91  -> $00 == $00 (branch not taken)
   [ok ] sta $FD91,#$40 / cmp $FD91 -> $40 == $40
   [ok ] sta $FD90,#$08 -> transmit started
   [ok ] wait loop  -> flag raised at Y=5 (timeout is Y=128)
   [ok ] sta $FD91,#$00 -> back to serial IN
   [ok ]                -> flag cleared, ready to receive
   [ok ]                -> receive bit counter aligned at 0

RESULT: burst-cpld.asm needs NO change
```

Czyli: **wystarczy nowy JED.** Zmiana ROMu parobka nie jest potrzebna.
