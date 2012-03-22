; Bootloader wersja 0.2
; Dyskietka identyfikuje siebie jako FAT12
; Bootloader samodzielnie przenosi siê do pamiêci
; Dodano skok do programu wczytanego z dyskietki

[BITS 16]		; Kod 16-bitowy
[ORG 0] 		; Przeskok wykonany zostanie póŸniej

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
	
	; SprawdŸ ile jest pamiêci konwencjonalnej 
	int 12h		; Przerwanie 12h pobiera iloœæ pamiêci konwencjonalnej
	shl ax,6	; Dokonaj konwersji iloœæ pamiêci na paragrafy
	
	; Rejestr AX zawiera teraz adres koñca pamiêci. Bêdziemy go u¿ywaæ do rezerwowania miejsca na poszczególne komórki
	
	; Rezerwujemy pamiêæ na bootsektor i na stos
	
	sub ax,32	; Rezerwujemy 512 bajtów na bootsektor (Paragraf ma 16 bajtów, a zatem potrzeba 512 / 16 = 32 paragrafy)
	
	; Ustalamy rejestry segmentowe
	mov     es, ax          ; Rejestr AX zawiera pozycjê segmentu
    sub     ax, 128	        ; Chcemy zarezerowowaæ 2048 bajtów na stos. To oznacza 128 paragrafów.
							; Od AX odejmujemy 128 paragrafów by obliczyæ pozycjê na której zacznie siê stos
    mov     ss, ax          ; Pocz¹tek stosu bêdzie tam gdzie jest AX
    mov     sp, 2048        ; Stos ma mieæ 2048 bajtów, tyle wiêc wpisujemy do rejestru SP
	
	; Przenosimy bootloader do pamiêci
	mov     cx, 256			; Bêdziemy przesuwaæ pamiêæ instrukcj¹ MOVSW czyli po 2 bajty naraz
							; to oznacza koniecznoœæ przeprowadzenia tej operacji 256 razy
    mov     si, 7C00h		; Bootloader musi byæ pod adresem pamiêci 7c00h
    xor     di, di			; Zerujemy rejestr DI
    mov     ds, di			; Wpisujemy do rejestru DS, wartoœæ DI (czyli zero)
    rep     movsw			; Przesuwamy blok pamiêci
	
	; Jeœli dosz³o do pomyœlnego przeniesienia do pamiêci wykonujemy skok
	
	; Odpowiednio manipuluj¹c rejestrami segmentowymi mo¿na "wmówiæ" aktualnie wykonywanemu kodowi
	; to, ¿e jest procedur¹ i nakazaæ powrót do punktu wywo³ania który ustalimy
	
	push    es			; Wrzucamy na stos rejestr ES
    push    word main	; Wrzucamy na stos po³o¿enie pocz¹tku programu
    retf				; Funkcja ta wykonana "powrót"

	; U¿ywaj¹c stosu mo¿na zamieniæ wartoœci rejestrów CS i DS
	
	main:
    push    cs						; Wrzucamy na stos rejestr CS
    pop     ds						; Œci¹gamy ze stosu wartoœæ i wrzucamy j¹ do rejestru DS
	
	; Teraz musimy zarezerwowaæ sobie miejsce na tablicê alokacji plików
	; Nastêpnie nale¿y j¹ wczytaæ ca³¹ do pamiêci
	
	; Obliczamy rozmiar tablicy alokacji plików
	mov 	ax,[bajtow_sektor]			; Ile jest bajtów na sektor?
	shr 	ax,4						; Przelicz rozmiar na paragrafy
	mov 	cx,sektorow_na_jednostke	; Ile sektorów zajmuje jednostka alokacji?
	mul 	cx							; Po wykonaniu mno¿enia przez CX rejestr AX ma w sobie rozmiar FAT

	; Skoro poznaliœmy ju¿ rozmiar FAT pora zarezerwowaæ mu miejsce w pamiêci
	mov     di, ss					; Jako rejestr DI (przeznaczenie) wybieramy rejestr SS (segmentowy)
    sub     di, ax					; Na podstawie uzyskanego rozmiaru FAT obliczamy miejsce gdzie mo¿emy go za³adowaæ
    mov     es, di					; Do rejestru ES wrzucamy wynik powy¿szego odejmowania. Mamy teraz adres naszego FAT
    xor     bx, bx                  ; Zerujemy rejestr BX, teraz mamy dok³adny adres miejsca na FAT
	
	; Mamy adres gdzie mo¿na przechowaæ ca³¹ tablicê FAT, mo¿na zaczaæ wczytywanie
	mov     ax, [ukryte_sektory1]		; Omijamy ukryte sektory
    mov     dx, [ukryte_sektory1+2]		; Omijamy ukryte sektory plus 2 bajty
    add     ax, [zarezerowane_sektory]	; Omijamy zarezerowane sektory
    adc     dx, bx                 	    ; Dodaj DX i BX z przeniesieniem, niezbêdne w LBA
    call    CzytajSektor				; Wywo³ujemy funkcje czytaj¹c¹ sektor
	
	
	; Po odczytaniu tablicy FAT i zamieszczeniu jej w pamiêci
	; Bierzemy siê za katalog g³ówny dysku i wczytujemy jego do pamiêci

    mov     bx, ax					; Wrzuæ do BX rejestr AX (pozycjê FAT)
    mov     di, dx                  ; Mo¿emy teraz poruszaæ siê po tablicy FAT

    mov     ax, 32					; Do AX daj 32 (dla MUL)
    mov     si, [limit_pozycji]		; SI - ile pozycji mo¿e byæ
    mul     si						; Ile pozycji razy 32 to rozmiar katalogu g³ównego
    div     word [bajtow_sektor]	; Podziel AX przez ilosc bajtów na sektor (uzyskamy rozmiar katalogu) 
    mov     cx, ax                  ; Rozmiar katalogu g³ównego w sektorach jest w CX

    mov     al, [ilosc_tablic_FAT]			; Ile mamy tablic FAT?
    cbw										; Ta instrukcja rozszerza rejestr AL do AX
    mul     word [sektorow_na_jednostke]	; Mno¿ymy iloœæ sektorów na jednostkê
    add     ax, bx					; Sumujemy wynik z BX
    adc     dx, di                  ; DX:DI to nasz adres logiczny
    push    es                      ; Wrzuæ segment FAT na stos
    push    word 60h				; Wrzucamy liczbê 60h na stos
    pop     es						; Zrzucamy liczbê 60h ze stosu do rejestru ES
    xor     bx, bx                  ; Zerujemy BX
	
	; Mamy teraz adres katalogu g³ównego w formacie ES:BX
	; Czytamy sektor
    call    CzytajSektor
	
	; Po przeczytaniu
	; Musimy zadbaæ o to by wskaŸniki do pamiêci by³y aktualne
	
    add     ax, cx					; Dodaæ do AX, CX
    adc     dx, bx                  ; Ustaw dane adresowania logicznego

    push    dx						; Na stos DX
    push    ax                      ; Na stos AX

	; Katalog g³ówny zosta³ zatem odczytany
	
	; Teraz pora na odnalezienie programu
	mov     di, bx                  ; Tablica plików w katalogu g³ównym
    mov     dx, si                  ; Ile jest wpisów w katalogu g³ównym
    mov     si, NazwaProgramu       ; Nazwa pliku który szukamy
	
	; Mamy nazwê pliku oraz wiemy gdzie szukaæ wpisu o jego istnieniu
	; Pora wiêæ na odszukanie pliku na dyskietce
	
	ZnajdzPlik:
	mov 	cx,11	;Nazwa ma dlugoœæ 11 znaków (standard 8.3)
	Szukanie:
	cmp		byte [es:di], ch	; Porównaj ze wzorcem
	je		Blad				; Jeœli to koniec katalogu
	pusha						; Wszystkie rejestry na stos
	repe	cmpsb				; Porównaj caly napis
	popa						; Wszystkie rejestry na stos
	je		Znaleziono
	add		di,32				; Jeœli pliku nie znaleziono to przechodzimy do nastêpnego wpisu
	dec		dx					; Zmniejszamy iloœæ wpisów do przeszukania
	jnz		Szukanie			; Sprawdzamy nastêpny
	Blad:
	hlt							; Gdy plik siê nie znajdzie zawieœ siê
	Znaleziono:
	mov		si,[es:di+1Ah]		; Gdy plik siê znajdzie ustawiamy rejestry segmentowe by wskazywaly na niego
	
	; ZnaleŸliœmy nasz plik
	; Teraz pora go za³adowaæ
	
	CzytajKolejnyKlaster:
	call	CzytajKlaster			; U¿yj funkcji
	cmp		si,0FF8h				; Czy to koniec pliku?
	jc		CzytajKolejnyKlaster	; Jeœli nie to czytaj nastêpny
	
	; Mamy nasz plik typu COM teraz zabieramy siê za jego obs³ugê
	
	; Tutaj dbamy o warunek DS=ES=SS=SP
	mov		ax,es					; Wstaw pozycjê kodu do AX
	sub		ax,10h					; ORG 100h
	mov		es,ax					; Ustaw rejestr segmentowy
	mov		ds,ax					; Ustaw rejestr segmentowy
	mov		ss,ax					; Ustaw rejestr segmentowy
	xor		sp,sp					; Zeruj wskaŸnik stosu
	
	push	es						; ES na stos
	push	word 100h				; wrzuæ 100h na stos
	jmp		Uruchomienie			; Uruchom program COM

	Uruchomienie:
    mov     dl, [cs:numer_dysku]    ; Sk¹d wystartowaliœmy?
    sti								; Przerwania w³¹cz
    retf							; "Powrót" do programu
									; Efektem bêdzie start pliku COM
	
;
; Funkcja czytaj¹ca klaster
;	
	CzytajKlaster:
	mov		bp,sp					; Do BP ³aduj wskaŸnik pocz¹tku stosu
	lea		ax,[si-2]				; £aduj adres do rejestru AX
	xor		ch,ch					; Zeruj rejestr CH
	mov		cl,[sektorow_na_klaster]; Do CL za³aduj iloœæ sektorów na klaster
	mul		cx						; Wylicz ile sektorów ma klaster
	
	add		ax,[ss:bp+2]			; Ustawiamy adresowanie logiczne
	adc		dx,[ss:bp+4]			;
	
	call	CzytajSektor			; Czytamy sektor
	
	mov		ax,[bajtow_sektor]		; Do AX ³aduj bajtów iloœæ bajtów na sektor
	shr		ax,4					; Ile paragrafów jest na sektor?
	mul		cx						; Oblicz ile przeczytaæ
	
	mov 	cx,es					;
	add		cx,ax					; Teraz rejestry segmentowe na
	mov		es,cx					; w³aœciwe pozycje
	
	mov		ax,3					;
	mul		si						; Tutaj obliczamy
	shr		ax,1					; adres dla rejestru
	xchg	ax,si					; bazowego (wskaŸnik na klaster)

	push	ds						; DS na stos
	mov		ds, [ss:bp+6]			; Kolejny segment FAT
	mov		si, [ds:si]				; Nastêpny klaster
	pop		ds
	
	jnc		CzytajKlaster2			; Skok jeœli klaster jest
									; parzysty
	
	shr		si,4

	CzytajKlaster2:
	and		si,0fffh				; Na³ó¿ maskê bitow¹
	
	KoniecCzytaniaKlastra:
	ret
;
; Koniec funkcji czytaj¹cej klaster
;


;
; Funkcja czytaj¹ca sektor
;
	
CzytajSektor:
        pusha			  ; Wrzuæ wszystkie rejestry na stos

CzytajSektorNastepny:
        mov     di, 5     ; Ile razy ponawiaæ odczyt

CzytajSektorPonow:
        pusha			  ; Wrzuæ wszystkie rejestry na stos
		
		; Na pocz¹tku musimy ustaliæ który sektor chcemy odczytaæ
		
		; Sektor
        div     word [sektorow_na_sciezke]	; Obliczamy numer sektora
        mov     cx, dx						; Do rejestru CX dajemy numer sektora
        inc     cx							; Zwiêkszamy ten numer o 1
		xor     dx, dx						; Zeruj rejestr DX
        
		; Cylinder
		div     word [ile_glowic]			; Obliczamy który cylinder nas interesuje
        mov     ch, al						; Do rejestru CH dajemy numer cylindra
		shl     ah, 6						; Rejestr AH mno¿ymy przez 64
        or      cl, ah						; Wykonynujemy operacjê OR na rejestrze CL
											; Wynikiem powy¿szego bêdzie prawie kompletny adres
		
		; G³owica
        mov     dh, dl				; Do DH wrzuæ DL (numer g³owycy)
        mov     dl, [numer_dysku]	; Do DL wrzuæ o który dysk chodzi

		; Teraz trzeba sformu³owaæ rozkaz dla BIOS do odczytu
		; Rejestr AL posiada informacjê ile sektorów
		; Rejestr AH wybiera funkcjê (2 oznacza odczyt)
		
		; Chcemy odczytaæ (2) jeden sektor (01)
        mov     ax, 201h

        int     13h                     ; Wykonaæ
		
		;Teraz podejmujemy kroki zale¿ne od rezultatu odczytu
        jnc     CzytajSektorKoniec      ; Flaga CF=0 brak b³êdu sektor odczytany
		;Jeœli pojawi siê b³¹d to
        xor     ah, ah                  ; Zeruj AH
        int     13h                     ; Przerwanie 13h gdy AH=0 resetuje dysk

        popa							; Przywróæ wartoœci rejestrów ze stosu
        dec     di						; Obni¿ ilosæ dozwolonych b³êdów
        jnz     CzytajSektorPonow       ; Próbój jeszcze raz
        hlt								; Zawieœ komputer jeœli siê nie uda³o

CzytajSektorKoniec:
        popa							; Przywróæ rejestry ze stosu
        dec     cx						; Mamy jeszcze o n-1 sektorów do oczytu
        jz      CzytajSektorKoniec2     ; Jeœli to ostatni to skacz do tej funkcji
		; Jeœli zaœ jest coœ do przeczytania
        add     bx, [bajtow_sektor] 	; Ustawiamy offset na nastêpny sektor
        add     ax, 1					; Zwiêksz rejestr AX o 1
        adc     dx, 0                   ; Ustaw LBA na nowy sektor
        jmp     short CzytajSektorNastepny	; Czytaj nastêpny

CzytajSektorKoniec2:
        popa		; Wszystko gotowe przywróæ pocz¹tkowe wartoœci rejestrów
        ret			; Wróæ do punktu wywo³ania

;		
; Koniec funkcji czytaj¹cej sektor	
;


; Sta³e u¿ywane w programie
NazwaProgramu 	db "SYSTEM  BIN"		;Program do wczytania	
napis 			db "Uruchamianie..."	;Napis do pokazania		
napiskoniec:	

;Bootsektor musi miec dokladnie 512 bajtow
times 510-($-$$) db 0 ;Zapelnij reszte miejsc zerami
dw 0xAA55			  ;Sygnatura identyfikujaca bootloader