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

"@M<#>A<nnnn><CR>", unde

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

4. Configurare numar pasi/rotatie

"@M<#>p<nnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnn> - reprezinta numarul de pasi necesari executiei unei rotatii, (6 caractere)

RASPUNS: "#EOK<CR>"


C. Mesaje de comanda

1. Setare numar pasi pentru a fi executati de motorul#

"@M<#>s<+/-><nnnnnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<+/-> -  reprezinta directia de deplasare (1 caracter)
<nnnnnnnnn> - reprezinta numarul de pasi ce se doreste a fi executat, (9 caractere)

SAU

1. Setare distanta de deplasare pentru incarcatura

"@M<#>d<+/-><nnn,nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<+/-> -  reprezinta directia de deplasare (1 caracter)
<nnnnnn,nn> - reprezinta distanta in mm cu 2 zecimale ce se doreste a fi parcursa de incarcatura, (6 caractere)

2. Setarea vitezei unghiulare in RPM cu care sa actioneze motorul#

"@M<#>w<nnnn,nn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnnnnn> - reprezinta valoarea vitezei unghiulare in RPM cu 2 zecimale, (7 caractere)

SAU

2. Setarea vitezei liniare in mm/s, a incarcaturii pe care sa o atinga incarcatura, fiind actionata de motorul#

"@M<#>v<nnnnnn><CR>", unde

<#> - reprezinta numarul motorului, (1 caracter)
<nnnnnn> - reprezinta valoarea vitezei liniare in mm/s cu 2 zecimale, (6 caractere)

!!!NOTE:
1. Legat de modul de functionare al motorului: Setarea acceleratiei unghiulare si liniare prin intermediul interfetei nu este posibila in cadrul acestei versiuni.

2. Aceasta versiune permite actionarea motorului la viteze prestabilite. Nu este posibila modificarea vitezei pe parcursul miscarii datorita unei probleme de timming
ce apare atunci cand vreau sa modific valoarea registrului RCR si apoi sa fac update la ceilalti registri ai timerului TIM1. Posibil sa apara un bug
atunci cand numarul de pasi ce se doreste a fi efectuat nu este multiplu de 250.

3.