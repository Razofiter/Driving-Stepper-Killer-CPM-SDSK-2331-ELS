In cadrul acestei versiuni sunt implementate urmatoarele:
- setarea numarului de pasi
- setarea directiei de deplasare
- setarea vitezei pentru deplasarea sub forma RPM
- comunicatia seriala pe USB cu utiilizatorul prin intermediu unei interfete grafice (vezi dosarul "GUI")
- TO BE CONTINUED...


Definire format mesaje:

A. Mesaje de status

"@PNG<CR>"

RASPUNS: "#EOK<CR>"

B. Mesaje de configurare

1. Configurare lungime axa liniara

"@M<#>L<nnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn> - reprezinta lungimea in mm a axei, (4 caractere)

2. Configurare lungime pas surub

"@M<#>P<nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nn> - reprezinta lungimea in mm a pasului surubului, (2 caractere)

3. Configurare masa incarcatura

"@M<#>m<nnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn> - reprezinta masa in g a incarcaturii, (4 caractere)

4. Configurare numar pasi/rotatie (rezolutia)

"@M<#>R<nnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn> - reprezinta numarul de pasi necesari executiei unei rotatii, (6 caractere)

RASPUNS: "#EOK<CR>"


C. Mesaje de comanda

1. Setare numar pasi pentru a fi executati de motorul#

"@M<#>S<+/-><nnnnnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<+/-> -  reprezinta directia de deplasare (1 caracter)
<nnnnnnnnn> - reprezinta numarul de pasi ce se doreste a fi executat, (9 caractere)

SAU

1. Setare distanta de deplasare pentru incarcatura

"@M<#>D<+/-><nnn,nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<+/-> -  reprezinta directia de deplasare (1 caracter)
<nnn,nn> - reprezinta distanta in mm cu 2 zecimale ce se doreste a fi parcursa de incarcatura, (6 caractere)

2. Setarea vitezei unghiulare in RPM cu care sa actioneze motorul#

"@M<#>W<nnnn,nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn,nn> - reprezinta valoarea vitezei unghiulare in RPM cu 2 zecimale, (7 caractere)

SAU

2. Setarea vitezei liniare in mm/s, a incarcaturii pe care sa o atinga incarcatura, fiind actionata de motorul#

"@M<#>V<nnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnn,nn> - reprezinta valoarea vitezei liniare in mm/s cu 2 zecimale, (6 caractere)

3. Setarea timpului de acceleratie unghiulara cu care sa fie actionat motorul#

"@M<#>-><nn,nnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nn,nnnn> - reprezinta valoarea timpului de acceleratie unghiulara in s cu 4 zecimale, (7 caractere)

4. Setarea timpului de deceleratie unghiulara cu care sa fie actionat motorul#

"@M<#>-<<nnnn,nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nn,nnnn> - reprezinta valoarea timpului de deceleratie unghiulara in s cu 4 zecimale, (7 caractere)

5. Setarea timpului total al miscarii pentru motorul#

"@M<#>E<nnnn,nnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn,nnnn> - reprezinta valoarea timpului total al miscarii in s cu 4 zecimale, (9 caractere)

6. Setarea intervalului de timp din timpul total al miscarii care va specifica modul de actionare al motorului#

"@M<#>I<nnnn,nnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn,nnnn> - reprezinta valoarea intervalului de timp in s cu 4 zecimale, (9 caractere)

D. Mesaje de PORNIRE/OPRIRE motor

1. Energizarea infasurarilor motorului# (ENABLE)

@M<#>START<CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)

2. Deenergizarea infasurarilor motorului# (DISABLE)

@M<#>STOP<CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)

3. Repetarea traiectoriei setate

"@M<#>x<nnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnn> - reprezinta numarul de repetari a traiectoriei confirgurate. Daca <nnn> = 0, atunci miscarea va fi executata la infinit.

4. Punerea in functiune a motorului

"@M<#>MOVE<CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)


!!!NOTE:
1. Legat de modul de functionare al motorului: Setarea acceleratiei unghiulare si liniare prin intermediul interfetei nu este posibila in cadrul acestei versiuni.

2. Aceasta versiune permite actionarea motorului la viteze prestabilite. Nu este posibila modificarea vitezei pe parcursul miscarii datorita unei probleme de timming
ce apare atunci cand vreau sa modific valoarea registrului RCR si apoi sa fac update la ceilalti registri ai timerului TIM1. Posibil sa apara un bug
atunci cand numarul de pasi ce se doreste a fi efectuat nu este multiplu de 250.

3. E o problema cu oprirea canalului PWM. Acesta trece in "1" logic si apoi cand il opresc genereaza inca un impuls. 
28.01.2019_UPDATE: Am gsit o medtoda "barbara" de rezolvare (vezi codul)

4. Se pare ca motorul trebuie oprit doar la incheierea programului de lucru. Altfel se pierd pasi intre sesiunile de PORNIRE-OPRIRE.

5. Trebuie sa selectam viteza/acceleratia motorului astfel incat valoarea perioadei semanlului corespunzator sa fie multiplu de 125ns
!!!! Aceasta observatie NU A FOST IMPLEMENTATA INCA!!!