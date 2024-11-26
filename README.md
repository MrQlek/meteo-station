# README

## Opis projektu

Projekt stacji pogodowej opartej na mikrokontrolerze **STM32L476** został przygotowany w celu przeprowadzenia warsztatów prezentujących proces programowania mikrokontrolerów z wykorzystaniem rejestrów. W projekcie wykorzystano dodatkowo następujące układy:

 - czujnik wilgotności, temperatury oraz ciśnienia - BME280,
 - analogowy czujnik światła - ALS-PT19,
 - wyświetlacz LCD 2x16 z konwerterem I2C LCM1602.

Kod znajdujący się w repozytorium będzie uzupełniany w trakcie trwania warsztatów.

## Przygotowanie do uruchomienia projektu

### Windows
 1. Zainstaluj WSL Ubuntu-22.04 (version 1)
 1. Zainstaluj [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
 1. Pobierz [ARM GNU TOOLCHAIN](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) - arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
 1. Wewnątrz WSL:  
    1. Utwórz w folderze domowym katalog `tools`
    1. Umieść w katalogu `tools` i rozpakuj wcześniej pobrany ARM GNU TOOLCHAIN
    1. Stwórz w katalogu `tools` link do wcześniej zainstalowanego programu ST `ln -s /mnt/c/<ścieżka do programu STM32CubeProgrammer>/STM32_Programmer_CLI.exe st-tool`
    1. Wyeksportuj zmienną środowiskową ARM_TOOLCHAIN_DIR ze ścieżką do plików wykonywalnych ARM GNU TOOLCHAIN `export ARM_TOOLCHAIN_DIR=/home/<username>/tools/arm-toolchain-13-3/bin/` (warto dodać do ~/.bashrc)
    1. Zainstaluj make `sudo apt install make`
    1. Zainstaluj minicom `sudo apt install minicom`
    1. Pobierz repozytorium `git clone <url_do_repozytorium>`
    1. Przejdź do pobranego repozytorium
    1. Wykonaj komendę `make` w celu zbudowania kodu
    1. Wykonaj komendę `make flash` w celu wgrania kodu na mikrokontroler

### Linux
 1. Zainstaluj make `sudo apt install make`
 1. Zainstaluj narzędzia do programowania mikrokontrolerów ST `sudo apt install stlink-tools`
 1. Zainstaluj minicom `sudo apt install minicom`
 1. Pobierz [ARM GNU TOOLCHAIN](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) - arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
 1. Rozpakuj pobrany toolchain i wyeksportuj zmienną środowiskową ARM_TOOLCHAIN_DIR  prowadzącą do jego plików wykonywalnych np. `export ARM_TOOLCHAIN_DIR=/home/<username>/tools/arm-toolchain-13-3/bin/`
1. Pobierz repozytorium `git clone <url_do_repozytorium>`
1. Przejdź do pobranego repozytorium
1. Wykonaj komendę `make` w celu zbudowania kodu
1. Wykonaj komendę `make flash` w celu wgrania kodu na mikrokontroler
