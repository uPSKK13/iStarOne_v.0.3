# iStarOne_v.0.3

Projekt systemu automatycznego sterowania opryskiwaczami sadowniczymi - strona serwera.
Tworzy publicznie rozgłaszaną sieć Wi-Fi, przez którą komunikuje się z klientem.
Użyty mikrokontroler ESP8266 wykorzystuje czujnik halla oraz przetwornik ciśnienia do odczytu kluczowych parametrów, które wysyła również do klienta.
W trybie autonomicznym mikrokontroler steruje elektrozaworami poprzez PWM, kontrolując ciśnienie, 
dzięki użytemu algorytmowi oraz mapie (x/y = delta ciśnienia/wypełnienie PWM)
