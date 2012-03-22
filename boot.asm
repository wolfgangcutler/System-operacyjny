; Bootloader wersja 0.2
; Dyskietka identyfikuje siebie jako FAT12
; Bootloader samodzielnie przenosi si� do pami�ci
; Dodano skok do programu wczytanego z dyskietki

[BITS 16]		; Kod 16-bitowy
[ORG 0] 		; Przeskok wykonany zostanie p�niej

; Musimy ominac BPB ktore nie jest kodem, a jedynie
; danymi identyfikujacymi dyskietke
; Rozkaz NOP jest konieczny by BPB znalazl sie we wlasciwym miejscu
jmp  start
nop
; BPB umozliwia obsluge dyskietki identyfikujac ja
; w systemie operacyjnym. Jest to niezbedne by moc
; na nia latwo zapisywac dane
; W celu ujednolicenia zapisu wszystkie liczby przeliczylem na system szesnastkowy
; ulatwia to pozniejsza obserwacje w hex-edytorze rezultatu asemblacji

nazwa_systemu         db "TestOS 1"	;Nazwa systemu operacyjnego (8 bajtow)
bajtow_sektor         dw 0x0200		;Ilosc bajtow na sektor na dyskietce jest to 512 bajtow (2 bajty)
sektorow_na_klaster   db 0x01		;Ilosc sektorow na jednostke alokacji ma dyskietce jest to 1 sektor na jednostke (1 bajt)
zarezerowane_sektory  dw 0x0001		;Ilosc sektorow zarezerwowanych my wymagamy 1 sektora (2 bajty)
ilosc_tablic_FAT      db 0x02		;Ilosc tablic alokacji FAT sa z reguly 2 (1 bajt)
limit_pozycji         dw 0x00E0		;Liczba pozycji w katalogu glownym, DOS ma ustawione to na 224 (2 bajty)
ile_sektorow          dw 0x0B40		;Liczba sektorow, na dyskietce mamy 1440 kb, w kazdym kilobajcie sa 2 sektory stad 2880 (2 bajty)
bajt_opisu_nosnika    db 0xF0		;Ten bajt opisuje typ nosnika, zgodnie ze standardem IBM dyskietka 1.44 MB ma wartosc 0F (1 bajt)
sektorow_na_jednostke dw 0x0009		;Ilosc sektorow na jednostke FAT, wartosc dla dyskietki 1.44MB wynosi 9 (2 bajty)
sektorow_na_sciezke   dw 0x0012		;Ilosc sektorow na sciezke dla dyskietki 3.5 1.44MB wynosi to 18 (2 bajty)
ile_glowic            dw 0x0002		;Liczba glowic, dla dyskietki 1.44MB sa to 2 glowice (2 bajty)
ukryte_sektory1       dd 0x00000000	;Ilosc sektorow ktore sa ukryte nie ukrywamy zadnych zatem wartosc jest 0 (4 bajty)
ukryte_sektory2       dd 0x00000000	;Ilosc sektorow ukrytych, jesli powyzej bylo 0 to tutaj tez musi byc (4 bajty
numer_dysku           db 0x00		;Numer dysku dla dyskietek jest to 0 (1 bajt)
flagi                 db 0x00		;Uzywane przez Windows NT do oznaczenia nosnika jako sprawny jesli jest 0 (1 bajt)
sygnatura             db 0x29		;Sygnatura dysku (1 bajt)
nr_seryjny            dd 0xFFFFFFFF	;Numer seryjny dysku, moze tu byc cokolwiek. Przyjalem wartosc 0 (2 bajty)
etykieta              db "TestOS 1.0 " ;Etykieta nosnika (11 bajtow)
system_plikow         db "FAT12   "    ;Rodzaj systemu plikow, w wypadku dyskietek jest to FAT12 0B (1 bajt)

start:
	; Zerujemy rejestr flag kierunkowych
	cld
	
	; Sprawd� ile jest pami�ci konwencjonalnej 
	int 12h		; Przerwanie 12h pobiera ilo�� pami�ci konwencjonalnej
	shl ax,6	; Dokonaj konwersji ilo�� pami�ci na paragrafy
	
	; Rejestr AX zawiera teraz adres ko�ca pami�ci. B�dziemy go u�ywa� do rezerwowania miejsca na poszczeg�lne kom�rki
	
	; Rezerwujemy pami�� na bootsektor i na stos
	
	sub ax,32	; Rezerwujemy 512 bajt�w na bootsektor (Paragraf ma 16 bajt�w, a zatem potrzeba 512 / 16 = 32 paragrafy)
	
	; Ustalamy rejestry segmentowe
	mov     es, ax          ; Rejestr AX zawiera pozycj� segmentu
    sub     ax, 128	        ; Chcemy zarezerowowa� 2048 bajt�w na stos. To oznacza 128 paragraf�w.
							; Od AX odejmujemy 128 paragraf�w by obliczy� pozycj� na kt�rej zacznie si� stos
    mov     ss, ax          ; Pocz�tek stosu b�dzie tam gdzie jest AX
    mov     sp, 2048        ; Stos ma mie� 2048 bajt�w, tyle wi�c wpisujemy do rejestru SP
	
	; Przenosimy bootloader do pami�ci
	mov     cx, 256			; B�dziemy przesuwa� pami�� instrukcj� MOVSW czyli po 2 bajty naraz
							; to oznacza konieczno�� przeprowadzenia tej operacji 256 razy
    mov     si, 7C00h		; Bootloader musi by� pod adresem pami�ci 7c00h
    xor     di, di			; Zerujemy rejestr DI
    mov     ds, di			; Wpisujemy do rejestru DS, warto�� DI (czyli zero)
    rep     movsw			; Przesuwamy blok pami�ci
	
	; Je�li dosz�o do pomy�lnego przeniesienia do pami�ci wykonujemy skok
	
	; Odpowiednio manipuluj�c rejestrami segmentowymi mo�na "wm�wi�" aktualnie wykonywanemu kodowi
	; to, �e jest procedur� i nakaza� powr�t do punktu wywo�ania kt�ry ustalimy
	
	push    es			; Wrzucamy na stos rejestr ES
    push    word main	; Wrzucamy na stos po�o�enie pocz�tku programu
    retf				; Funkcja ta wykonana "powr�t"

	; U�ywaj�c stosu mo�na zamieni� warto�ci rejestr�w CS i DS
	
	main:
    push    cs						; Wrzucamy na stos rejestr CS
    pop     ds						; �ci�gamy ze stosu warto�� i wrzucamy j� do rejestru DS
	
	; Teraz musimy zarezerwowa� sobie miejsce na tablic� alokacji plik�w
	; Nast�pnie nale�y j� wczyta� ca�� do pami�ci
	
	; Obliczamy rozmiar tablicy alokacji plik�w
	mov 	ax,[bajtow_sektor]			; Ile jest bajt�w na sektor?
	shr 	ax,4						; Przelicz rozmiar na paragrafy
	mov 	cx,sektorow_na_jednostke	; Ile sektor�w zajmuje jednostka alokacji?
	mul 	cx							; Po wykonaniu mno�enia przez CX rejestr AX ma w sobie rozmiar FAT

	; Skoro poznali�my ju� rozmiar FAT pora zarezerwowa� mu miejsce w pami�ci
	mov     di, ss					; Jako rejestr DI (przeznaczenie) wybieramy rejestr SS (segmentowy)
    sub     di, ax					; Na podstawie uzyskanego rozmiaru FAT obliczamy miejsce gdzie mo�emy go za�adowa�
    mov     es, di					; Do rejestru ES wrzucamy wynik powy�szego odejmowania. Mamy teraz adres naszego FAT
    xor     bx, bx                  ; Zerujemy rejestr BX, teraz mamy dok�adny adres miejsca na FAT
	
	; Mamy adres gdzie mo�na przechowa� ca�� tablic� FAT, mo�na zacza� wczytywanie
	mov     ax, [ukryte_sektory1]		; Omijamy ukryte sektory
    mov     dx, [ukryte_sektory1+2]		; Omijamy ukryte sektory plus 2 bajty
    add     ax, [zarezerowane_sektory]	; Omijamy zarezerowane sektory
    adc     dx, bx                 	    ; Dodaj DX i BX z przeniesieniem, niezb�dne w LBA
    call    CzytajSektor				; Wywo�ujemy funkcje czytaj�c� sektor
	
	
	; Po odczytaniu tablicy FAT i zamieszczeniu jej w pami�ci
	; Bierzemy si� za katalog g��wny dysku i wczytujemy jego do pami�ci

    mov     bx, ax					; Wrzu� do BX rejestr AX (pozycj� FAT)
    mov     di, dx                  ; Mo�emy teraz porusza� si� po tablicy FAT

    mov     ax, 32					; Do AX daj 32 (dla MUL)
    mov     si, [limit_pozycji]		; SI - ile pozycji mo�e by�
    mul     si						; Ile pozycji razy 32 to rozmiar katalogu g��wnego
    div     word [bajtow_sektor]	; Podziel AX przez ilosc bajt�w na sektor (uzyskamy rozmiar katalogu) 
    mov     cx, ax                  ; Rozmiar katalogu g��wnego w sektorach jest w CX

    mov     al, [ilosc_tablic_FAT]			; Ile mamy tablic FAT?
    cbw										; Ta instrukcja rozszerza rejestr AL do AX
    mul     word [sektorow_na_jednostke]	; Mno�ymy ilo�� sektor�w na jednostk�
    add     ax, bx					; Sumujemy wynik z BX
    adc     dx, di                  ; DX:DI to nasz adres logiczny
    push    es                      ; Wrzu� segment FAT na stos
    push    word 60h				; Wrzucamy liczb� 60h na stos
    pop     es						; Zrzucamy liczb� 60h ze stosu do rejestru ES
    xor     bx, bx                  ; Zerujemy BX
	
	; Mamy teraz adres katalogu g��wnego w formacie ES:BX
	; Czytamy sektor
    call    CzytajSektor
	
	; Po przeczytaniu
	; Musimy zadba� o to by wska�niki do pami�ci by�y aktualne
	
    add     ax, cx					; Doda� do AX, CX
    adc     dx, bx                  ; Ustaw dane adresowania logicznego

    push    dx						; Na stos DX
    push    ax                      ; Na stos AX

	; Katalog g��wny zosta� zatem odczytany
	
	; Teraz pora na odnalezienie programu
	mov     di, bx                  ; Tablica plik�w w katalogu g��wnym
    mov     dx, si                  ; Ile jest wpis�w w katalogu g��wnym
    mov     si, NazwaProgramu       ; Nazwa pliku kt�ry szukamy
	
	; Mamy nazw� pliku oraz wiemy gdzie szuka� wpisu o jego istnieniu
	; Pora wi�� na odszukanie pliku na dyskietce
	
	ZnajdzPlik:
	mov 	cx,11	;Nazwa ma dlugo�� 11 znak�w (standard 8.3)
	Szukanie:
	cmp		byte [es:di], ch	; Por�wnaj ze wzorcem
	je		Blad				; Je�li to koniec katalogu
	pusha						; Wszystkie rejestry na stos
	repe	cmpsb				; Por�wnaj caly napis
	popa						; Wszystkie rejestry na stos
	je		Znaleziono
	add		di,32				; Je�li pliku nie znaleziono to przechodzimy do nast�pnego wpisu
	dec		dx					; Zmniejszamy ilo�� wpis�w do przeszukania
	jnz		Szukanie			; Sprawdzamy nast�pny
	Blad:
	hlt							; Gdy plik si� nie znajdzie zawie� si�
	Znaleziono:
	mov		si,[es:di+1Ah]		; Gdy plik si� znajdzie ustawiamy rejestry segmentowe by wskazywaly na niego
	
	; Znale�li�my nasz plik
	; Teraz pora go za�adowa�
	
	CzytajKolejnyKlaster:
	call	CzytajKlaster			; U�yj funkcji
	cmp		si,0FF8h				; Czy to koniec pliku?
	jc		CzytajKolejnyKlaster	; Je�li nie to czytaj nast�pny
	
	; Mamy nasz plik typu COM teraz zabieramy si� za jego obs�ug�
	
	; Tutaj dbamy o warunek DS=ES=SS=SP
	mov		ax,es					; Wstaw pozycj� kodu do AX
	sub		ax,10h					; ORG 100h
	mov		es,ax					; Ustaw rejestr segmentowy
	mov		ds,ax					; Ustaw rejestr segmentowy
	mov		ss,ax					; Ustaw rejestr segmentowy
	xor		sp,sp					; Zeruj wska�nik stosu
	
	push	es						; ES na stos
	push	word 100h				; wrzu� 100h na stos
	jmp		Uruchomienie			; Uruchom program COM

	Uruchomienie:
    mov     dl, [cs:numer_dysku]    ; Sk�d wystartowali�my?
    sti								; Przerwania w��cz
    retf							; "Powr�t" do programu
									; Efektem b�dzie start pliku COM
	
;
; Funkcja czytaj�ca klaster
;	
	CzytajKlaster:
	mov		bp,sp					; Do BP �aduj wska�nik pocz�tku stosu
	lea		ax,[si-2]				; �aduj adres do rejestru AX
	xor		ch,ch					; Zeruj rejestr CH
	mov		cl,[sektorow_na_klaster]; Do CL za�aduj ilo�� sektor�w na klaster
	mul		cx						; Wylicz ile sektor�w ma klaster
	
	add		ax,[ss:bp+2]			; Ustawiamy adresowanie logiczne
	adc		dx,[ss:bp+4]			;
	
	call	CzytajSektor			; Czytamy sektor
	
	mov		ax,[bajtow_sektor]		; Do AX �aduj bajt�w ilo�� bajt�w na sektor
	shr		ax,4					; Ile paragraf�w jest na sektor?
	mul		cx						; Oblicz ile przeczyta�
	
	mov 	cx,es					;
	add		cx,ax					; Teraz rejestry segmentowe na
	mov		es,cx					; w�a�ciwe pozycje
	
	mov		ax,3					;
	mul		si						; Tutaj obliczamy
	shr		ax,1					; adres dla rejestru
	xchg	ax,si					; bazowego (wska�nik na klaster)

	push	ds						; DS na stos
	mov		ds, [ss:bp+6]			; Kolejny segment FAT
	mov		si, [ds:si]				; Nast�pny klaster
	pop		ds
	
	jnc		CzytajKlaster2			; Skok je�li klaster jest
									; parzysty
	
	shr		si,4

	CzytajKlaster2:
	and		si,0fffh				; Na�� mask� bitow�
	
	KoniecCzytaniaKlastra:
	ret
;
; Koniec funkcji czytaj�cej klaster
;


;
; Funkcja czytaj�ca sektor
;
	
CzytajSektor:
        pusha			  ; Wrzu� wszystkie rejestry na stos

CzytajSektorNastepny:
        mov     di, 5     ; Ile razy ponawia� odczyt

CzytajSektorPonow:
        pusha			  ; Wrzu� wszystkie rejestry na stos
		
		; Na pocz�tku musimy ustali� kt�ry sektor chcemy odczyta�
		
		; Sektor
        div     word [sektorow_na_sciezke]	; Obliczamy numer sektora
        mov     cx, dx						; Do rejestru CX dajemy numer sektora
        inc     cx							; Zwi�kszamy ten numer o 1
		xor     dx, dx						; Zeruj rejestr DX
        
		; Cylinder
		div     word [ile_glowic]			; Obliczamy kt�ry cylinder nas interesuje
        mov     ch, al						; Do rejestru CH dajemy numer cylindra
		shl     ah, 6						; Rejestr AH mno�ymy przez 64
        or      cl, ah						; Wykonynujemy operacj� OR na rejestrze CL
											; Wynikiem powy�szego b�dzie prawie kompletny adres
		
		; G�owica
        mov     dh, dl				; Do DH wrzu� DL (numer g�owycy)
        mov     dl, [numer_dysku]	; Do DL wrzu� o kt�ry dysk chodzi

		; Teraz trzeba sformu�owa� rozkaz dla BIOS do odczytu
		; Rejestr AL posiada informacj� ile sektor�w
		; Rejestr AH wybiera funkcj� (2 oznacza odczyt)
		
		; Chcemy odczyta� (2) jeden sektor (01)
        mov     ax, 201h

        int     13h                     ; Wykona�
		
		;Teraz podejmujemy kroki zale�ne od rezultatu odczytu
        jnc     CzytajSektorKoniec      ; Flaga CF=0 brak b��du sektor odczytany
		;Je�li pojawi si� b��d to
        xor     ah, ah                  ; Zeruj AH
        int     13h                     ; Przerwanie 13h gdy AH=0 resetuje dysk

        popa							; Przywr�� warto�ci rejestr�w ze stosu
        dec     di						; Obni� ilos� dozwolonych b��d�w
        jnz     CzytajSektorPonow       ; Pr�b�j jeszcze raz
        hlt								; Zawie� komputer je�li si� nie uda�o

CzytajSektorKoniec:
        popa							; Przywr�� rejestry ze stosu
        dec     cx						; Mamy jeszcze o n-1 sektor�w do oczytu
        jz      CzytajSektorKoniec2     ; Je�li to ostatni to skacz do tej funkcji
		; Je�li za� jest co� do przeczytania
        add     bx, [bajtow_sektor] 	; Ustawiamy offset na nast�pny sektor
        add     ax, 1					; Zwi�ksz rejestr AX o 1
        adc     dx, 0                   ; Ustaw LBA na nowy sektor
        jmp     short CzytajSektorNastepny	; Czytaj nast�pny

CzytajSektorKoniec2:
        popa		; Wszystko gotowe przywr�� pocz�tkowe warto�ci rejestr�w
        ret			; Wr�� do punktu wywo�ania

;		
; Koniec funkcji czytaj�cej sektor	
;


; Sta�e u�ywane w programie
NazwaProgramu 	db "SYSTEM  BIN"		;Program do wczytania	
napis 			db "Uruchamianie..."	;Napis do pokazania		
napiskoniec:	

;Bootsektor musi miec dokladnie 512 bajtow
times 510-($-$$) db 0 ;Zapelnij reszte miejsc zerami
dw 0xAA55			  ;Sygnatura identyfikujaca bootloader