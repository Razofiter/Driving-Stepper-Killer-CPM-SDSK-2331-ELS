clc
%% Calculator de parametri utili in vederea generarii cuplului maxim pentru
%  a realiza o miscare de rotatie a unei sarcini de forma unui disc, pentru
% care se cunosc caracteristicile.
%%
% Servomotor utilizat: CPM-SDSK-2331S-ELS
% Link: https://www.teknic.com/model-info/CPM-SDSK-2331S-ELS/

%% Constante/date de intrare:
% A. Motor
putereMaxMot = 193;        % [W]  
putereNomMot = 193;        % [W]
vMaxMot = 2520;     % [RPM]
m_max = 4.4;     %[Nm]   Aceasta valoare este valabila pana la aprox. 400RPM
m_nom = 0.9;     %[Nm]   Aceasta valoare este valabila pana la aprox. 2000RPM
nrPasiMot = 6400;    %[NaN]
inchToMm = 25.4;       %[mm]
diamAxMot = 1/4*inchToMm;   %[mm]
rezMot = 360/nrPasiMot;    %[grade]
NrImpulsuriRotatie = [200,400,800,1600,3200,6400,12800,25600,51200];    %[imp/R]
ras = 0.0044;           %[s]
NrPasiImpuls = nrPasiMot./NrImpulsuriRotatie;  %[pasi/imp]
% Cunoastem viteza maxima a motorului, deci putem calcula cat timp dureaza
% efectuarea unei rotatii
minuteToMicrosec = 60000000;    %[us]
minTimeToPerformOneRot = minuteToMicrosec/vMaxMot; % [us]
perImpulsMin = minTimeToPerformOneRot./NrImpulsuriRotatie;
% Vom aproxima la cel mai apropiat intreg si vom tine cont ca perioada
% minima a unui impuls care a rezultat in urma testarilor la 6400 de
% impulsuri/rotatie este de 4us
for i=1:length(perImpulsMin)
    perImpulsMin(i) = round(perImpulsMin(6),0)*NrPasiImpuls(i);
    if perImpulsMin(i) < round(perImpulsMin(6),0)
        perImpulsMin(i) = round(perImpulsMin(6),0);
    end
end

% B. Structura
masaSarcina = 1.322;     % [kg]
diamDisc = 0.09816;     % [m]
pasSurub = [2,5,10,20];     %[mm]


%%
% Vom determina acceleratia unghiulara maxima, careia ii va corespunde
% frecventa maxima a semnalului de intrare (de tip impuls), necesar
% pentru comanda motorului, in conditiile date:

% ------- Cuplu(M) = MomInertie(J)*accUnghi (1) ---------------- 
% * Cunoastem valoarea cuplului pe care o poate dezvolta motorul
% * Vom calcula momentul de inertie al sarcinii ambarcate pentru a
% determina acceleratia unghiulara

% ------- MomInertie[J] = 1/2(masaSarcina*r^2) (2) ---------
% * Cunoastem valoarea masei si a razei pentru discul considerat
j = 0.5*(masaSarcina*(diamDisc/2)^2); % [Kg*m^2]

% Inlocuim J in relatia (1) si obtinem:
accUnghi_Mmax = m_max/j;    %[rad/s^2]
accUnghi_Mnom = m_nom/j;     %[rad/s^2]

accUnghi_Mmax_RPM = (accUnghi_Mmax/(2*pi))*60;   % [RPM/s]
accUnghi_Mnom_RPM = (accUnghi_Mnom/(2*pi))*60;   % [RPM/s]
% vezi https://www.calculator.org/properties/angular_acceleration.html

%% Pana in acest punct am calculat care ar fi limitarea motorului
% (aceeleratia unghiulara maxima),d.p.d.v dinamic.
% In continuare vom defini cum dorim sa arate semnalul de excitatie si vom
% verifica daca cerintele impuse in ceea ce priveste amplitudinea si
% frecventa semnalului implicat, nu depasesc conditiile de operare la
% limita, ale motorului ales.

% * Ne propunem sa generam o frecventa a oscilatiilor in plaja 5-20Hz. Motivul
% pentru care selectam acest lucru se regaseste in materialul:
%%
% https://books.google.ro/books?id=KLnMBQAAQBAJ&pg=PA235&lpg=PA235&dq=
% low+frequency+high+amplitude+vibrations+of+aircraft+wings&source=bl&ots=
% f-Ja91I2jv&sig=ACfU3U1nl7GyvwypQRGHoahMRmw7iVhRCA&hl=en&sa=X&ved=
% 2ahUKEwidrK6dm8_hAhVK3KQKHU-YDM8Q6AEwAHoECAgQAQ#v=onepage&q=
% low%20frequency%20high%20amplitude%20vibrations%20of%20aircraft%20wings&f=false
%%
% * Vom selecta diferite amplitudini, dar si diferite frecvente in plaja
% 5-20 Hz pentru a obtine o serie de rezultate. Pornim de la expresia
% generala a unui semnal sinusoidal/cosinusoidal:
% ------- x(t) = asin(wt+phi) (3)---------
%           .
% ------- x(t) = awcos(wt+phi) (4)-----
%          ..
% ------- x(t) = -aw^2sin(wt+phi) (5)-----
%         ...
% ------- x(t) = -aw^3cos(wt+phi) (6)-----
% * a[rotatii] = numarul de rotatii care trebuie executat, unde a=x/p.
% x este deplasarea de translatie, iar passSurub.
% * w[rad/s] = viteza unghiulara a axului motorului sau a sarcinii (deoarece
% aceasta este prinsa de axul motorului
% * t[s] = timpul.
% * phi[rad] = defazajul in timp

%% Exemplul nr. I
% ------- x(t) = 2/p*sin(2*pi*10*t) -----

% Asadar, dorim sa generam un semnal de excitatie care sa genereze o
% deplasare(in modul) de 2 mm, la o frecventa de 10Hz. Defazajul este 0.
% CERINTE:
% Pentru a realiza aceasta miscare este necesar sa determinam:
% a) cate impulsuri trebuie generate pentru a executa deplasarea de 2mm
% b) care este viteza unghiulara maxima la care se va ajunge
% c) care este acceleratia unghiulara necesara pentru a indeplini
% conditiile a) si b)
% REZOLVARE:
% a) Deplasarea de 2mm depinde in primul rand de pasul surubului din cadrul
% montajului. Astfel ca numarul de rotatii care rezulta este:
x = 2; %[mm]
rot = x./pasSurub; %[R]
NrImpuls = rot'*NrImpulsuriRotatie; % matrice 4x9 ce contine pe fiecare linie
% cate impulsuri trebuie executate atunci cand valoarea pasului surubului
% este aplicata fiecarui caz de numar de impulsuri/rotatie.

% b) Viteza unghiulara[w] este data de frecventa vibratiilor,
% f = rotatii/sec. Dar numarul de rotatii depinde de pasul surubului
f = 10;     %[Hz]
% Aplicam (4) si tinand cont ca valoarea lui cos este maxim 1, obtinem:
w_max = 2*pi*f*rot;    %[rad/s]
vMot_max = f*rot*60;    %[RPM]
% Verificam ca viteza unghiulara obtinuta sa nu depaseasca valoarea maxima.
% Daca valoarea este buna, calculam care este freventa semnalului
% dreptunghiular care trebuie aplicat, altfel punem 0!
perImpuls = ones(length(vMot_max),length(NrImpulsuriRotatie));
fSemnalDreptunghiular = ones(length(vMot_max),length(NrImpulsuriRotatie));
for j=1:length(vMot_max) 
    if vMot_max(j) < vMaxMot
        timeToPerformOneRot = minuteToMicrosec/vMot_max(j); % [us]
        perImpuls(j,:) = timeToPerformOneRot./NrImpulsuriRotatie;    %[us]
        fSemnalDreptunghiular(j,:) = (1./perImpuls(j,:));   %[MHz]
    else
    vMot_max(j) = 0;
    end
end
% c) Aplicam (5) si tinand cont ca valoarea lui sinus este max 1, obtinem:
accUnghi_max= rot*(2*pi*f)^2;      %[rad/s^2]

% Verificam daca acceleratia maxima a fost depasita. Actualizam rezultatele
for i = 1:length(accUnghi_max)
    if accUnghi_max(i) >= accUnghi_Mmax
        sprintf('Aceleratia de %0.2f rad/s^2 depaseste valoarea maxim acceptata de %0.02f rad/s^2!\r',accUnghi_max(i),accUnghi_Mmax)
        sprintf('Aceasta acceleratie este obtinuta atunci cand pasul surubului este confirgurat sa fie de %0d mm',pasSurub(i)) 
    end
end


%%
% Reprezentam grafic variatia in timp a pozitiei,vitezei si acceleratiei
% unghiulare.
% Alegem incrementul de timp ca fiind egal cu perioada minima a semnalului
% dreptunghiular de comanda.
t = [];
index = 1;
len_l = length(perImpuls(1,:));
len_k = length(perImpuls(:,1));
% Perioada minima a semnalului dreptunghiular ce trebuie aplicat, pentru
% controlul motorului, trebuie sa fie numar intreg pozitiv pentru a face
% posibile implementarea software.
% Valoarea perioadei este exprimata in microsecunde. Prin impartirea la
% 10^6, ne propunem ca rezultatul sa fie de ordinul secundelor.
rounded_perImpuls = round(round(perImpuls)/10^6,6);
for k=1:len_k
    for l = 1:len_l
        % Pentru fiecare element al matriciei rounded_perImpuls, obtinem un
        % vector linie cu numarul de perioade/impulsuri 
        incr = 0:rounded_perImpuls(k,l):0.1;
        % La fiecare iteratie, (la parcurgerea fiecarui element al matricei
        % rounded_perImpuls obtinem un vector linie pe care il adaugam la
        % matricea de tim "t".
        % Vom obtine asadar o matrice egala cu dimensiunea:
        
        % ^^Nr. linii = nr. variante pas surub (pasSurub)* nr de variante de
        % nr de impulsuri pentru a executa o rotatie (NrImpulsuriRotatie)
        % ^^Nr. coloane = variabil, functie de valoarea incrementului de
        % timp obtinut in matricea rounded_perImpuls. Acesta va constitui
        % pasul de timp de la valoarea 0 la valoarea corespunzatoare perioadei
        % de timp a semnalului sinusoidal. Numarul de pasi pentru ficare
        % caz in parte va genera nr de coloane corespunzator fiecarei linii
        t(index,1:length(incr)) = incr;
        index = index+1;
    end
end

% Remove zeros from matrix t
% See
% https://www.mathworks.com/matlabcentral/answers/330859-how-to-remove-zeros-from-an-array
% for more info
[sz1,sz2]=size(t);
% Acest vector tine dimensiunea fiecarei linii, dupa inlaturarea valorilor
% de 0
new_sz2=zeros(1,sz1);      
for i=1:sz1
    new_mat{i}= nonzeros(t(i,:));
    new_row_sz(i)=size(new_mat{i},1);
end
if range(new_row_sz) == 0
    new_mat=cell2mat(new_mat);
end

% Pentru a realiza graficele de pozitie,viteza si acceleratie este nevoie sa
% fragmentam vectorul new_mat, functie de numarul de variante de pas de
% surub. Acest lucru este necesar pentru ca vom avea un numar de grafice
% (functie de numarul de impulsuri necesare pentru a executa o rotatie
% mecanica), pentru fiecare configuratie data de pasul surubului.
nrConf = length(new_mat)/length(rot);

figure(1)
subplot(4,1,1)
hold on
for ix=1:length(rot)
    for iy=((ix-1)*nrConf+1):(nrConf*ix)
        x_t = rot(ix)*sin(2*pi*f*new_mat{iy}+pi/2);
        plot(new_mat{iy},x_t)
    end
end
xlabel('t[s]','FontWeight','bold')
ylabel('x[rad]','FontWeight','bold')
title( 'Variatia in timp a pozitiei unghiulare')
hold off

subplot(4,1,2)
hold on
for ix=1:length(rot)
    for iy=((ix-1)*nrConf+1):(nrConf*ix)
        dx_t = rot(ix)*2*pi*f*cos(2*pi*f*new_mat{iy}+pi/2);
        plot(new_mat{iy},dx_t)
    end
end
xlabel('t[s]','FontWeight','bold')
ylabel('dx[rad/s]','FontWeight','bold')
title( 'Variatia in timp a vitezei unghiulare')
hold off

subplot(4,1,3)
hold on
for ix=1:length(rot)
    for iy=((ix-1)*nrConf+1):(nrConf*ix)
        ddx_t = -rot(ix)*(2*pi*f)^2*sin(2*pi*f*new_mat{iy}+pi/2);
        plot(new_mat{iy},ddx_t)
    end
end
xlabel('t[s]','FontWeight','bold')
ylabel('ddx[rad/s^2]','FontWeight','bold')
title( 'Variatia in timp a acceleratiei unghiulare')
hold off

subplot(4,1,4)
hold on
for ix=1:length(rot)
    for iy=((ix-1)*nrConf+1):(nrConf*ix)
        dddx_t = -rot(ix)*(2*pi*f)^3*cos(2*pi*f*new_mat{iy}+pi/2);
        plot(new_mat{iy},dddx_t)
    end
end
xlabel('t[s]','FontWeight','bold')
ylabel('ddx[rad/s^3]','FontWeight','bold')
title( 'Variatia derivatei acceleratiei unghiulare in raport cu timpul')
hold off
%%

% Luam un exemplu concret din cele evidentiate mai sus pe baza caruia dorim
% sa obtinem date de intrare in vederea determinarii caracteristilor
% semnalului dreptunghiular de comanda.
% Asadar, vom avea:

% - nr. impulsuri/rotatie = 6400
% - pas surub = 10 mm

% Determinam vetorul vitezei unghiulare folosind ca increment de timp o
% valoare care sa corespunda numarului de pasi care trebuie executat,
% respectiv 6400 rotatie.
% Vom avea:
steps_in_relation_to_distance = rot'*NrImpulsuriRotatie;

% Vom imparti perioada vibratiei (T sau 1/f) in 4 intervale pentru alicarea
% formulei (4). Facem acest lucru deoarece dorim sa bifam extremele vitezei
f = 10;
valoare_dx_t = [];
valoare_dx_t_RPM = [];
pwmPeriod = [];

%% Evaluam pentru primul interval, adica T/4.
% Vom incerca sa realizam toate cazurile posibile(functie de pasul surubului
% si de nr de impulsuri pe rot)

% Calculam care este perioada de timp pe care ar trebui sa o aiba semnalul
% dreptunghiular, la un timp egal cu ras fata de momentul la care viteza
% este 0.
% Am comentat bucata de code de mai jos in ideea in care am ales o solutie
% mai simpla deocamdata. Anume, voi folosi perioada semnalului
% dreptunghiular care imi rezulta la jumatate din ras.
% for i = 1:length(rot)
%     for j = 1:length(NrImpulsuriRotatie)
%         vel_acoording_to_RAS_interval = rot(i)*2*pi*f*cos(2*pi*f*ras+ pi/2);  % [rad/s]
%         vel_acoording_to_RAS_interva_RPM = rot(i)*2*pi*f*cos(2*pi*f*ras+ pi/2)*60/2/pi;   %[RPM]
%         % Obtinem valorile in modul
%         Ras_pwmPeriod(i,j) = abs(round(60000000/vel_acoording_to_RAS_interva_RPM/NrImpulsuriRotatie(j)));
%     end
% end

% In continuare, vom calcula care este perioada semnalului la jumatate din
% ras si vom determina pasul de timp (variabil) pana la T/4
for i = 1:length(rot)
    for j = 1:length(NrImpulsuriRotatie)
        vel_acoording_to_semiRAS_interval = rot(i)*2*pi*f*cos(2*pi*f*ras/2+ pi/2);  % [rad/s]
        vel_acoording_to_semiRAS_interva_RPM = rot(i)*2*pi*f*cos(2*pi*f*ras/2+ pi/2)*60/2/pi;   %[RPM]
        % Obtinem valorile in modul
        SemiRas_pwmPeriod(i,j) = abs(round(60000000/vel_acoording_to_semiRAS_interva_RPM/NrImpulsuriRotatie(j)));
    end
end

incr = (ras/2+SemiRas_pwmPeriod*10^(-6));
for r = 1: length(rot)
    for c = 1:length(NrImpulsuriRotatie)
        pos = 1;
        while incr(r,c) <= (1/f/4)
        %while incr(r,c) <= (1/f/4)
            valoare_dx_t((r-1)*length(NrImpulsuriRotatie)+c,pos) = rot(r)*2*pi*f*cos(2*pi*f*incr(r,c)+ pi/2);  % [rad/s]
            valoare_dx_t_RPM = rot(r)*2*pi*f*cos(2*pi*f*incr(r,c)+ pi/2)*60/2/pi;   %[RPM]
            % Obtinem valorile in modul
            pwmPeriod((r-1)*length(NrImpulsuriRotatie)+c,pos) = abs(round(60000000/valoare_dx_t_RPM/NrImpulsuriRotatie(c)));
            
            time_plot((r-1)*length(NrImpulsuriRotatie)+c,pos) = incr(r,c);
            % Ne asiguram ca toate valorile sa nu depaseasca valoarea |65535|:
            if pwmPeriod((r-1)*length(NrImpulsuriRotatie)+c,pos) > 65535
                pwmPeriod((r-1)*length(NrImpulsuriRotatie)+c,pos) = 65535;
            end
            incr(r,c) = incr(r,c)+pwmPeriod((r-1)*length(NrImpulsuriRotatie)+c,pos)/10^6;
            pos = pos+1;
        end
    end
end

% Remove zeros from matrix pwmPeriod and from time_plot
% See
% https://www.mathworks.com/matlabcentral/answers/330859-how-to-remove-zeros-from-an-array
% for more info
[sz1,sz2]=size(pwmPeriod);
% Acest vector tine dimensiunea fiecarei linii, dupa inlaturarea valorilor
% de 0
new_sz2=zeros(1,sz1);      
for i=1:sz1
    new_pwmPeriod_mat{i}= nonzeros(pwmPeriod(i,:));
    new_plot_time_mat{i} = nonzeros(time_plot(i,:));
    new_valoare_dx_t_mat{i} = nonzeros(valoare_dx_t(i,:));
    new_row_sz(i)=size(new_pwmPeriod_mat{i},1);
end
if range(new_row_sz) == 0
    new_pwmPeriod_mat=cell2mat(new_pwmPeriod_mat);
    new_plot_time_mat = cell2mat(new_plot_time_mat);
    new_valoare_dx_t_mat=cell2mat(new_valoare_dx_t_mat);
end

% Verificam daca putem face amplitudinea specificata in cerinta
for r = 1: length(rot)
    for c = 1:length(NrImpulsuriRotatie)
        if length(new_pwmPeriod_mat{(r-1)*length(NrImpulsuriRotatie)+c}) > steps_in_relation_to_distance(r,c)
            sprintf('Configuratia %d,%d este buna!\r',r,c)
        end
    end
end

% Plotam un caz pentru a observa discretizarea graficului de viteza functie
% de pasul de timp variabil obtinut, corespunzator fiecarei configuratii de
% nr. de impulsuri/rotatie.
% poate  
figure(2)
hold on
for ix=1:length(rot)
    for iy=((ix-1)*nrConf+1):(nrConf*ix)
%         dx_t = rot(ix)*2*pi*f*cos(2*pi*f*new_mat{iy}+pi/2);
%         plot(new_mat{iy},dx_t,new_plot_time_mat{iy},new_valoare_dx_t_mat{iy})
        plot(new_plot_time_mat{iy},new_valoare_dx_t_mat{iy})
    end
end
xlabel('t[s]','FontWeight','bold')
ylabel('dx[rad/s]','FontWeight','bold')
title( 'Variatia in timp a vitezei unghiulare conform pasului de timp variabil calculat')
hold off

% Construim o matrice in care salvam pe fiecare linie de cate ori se repeta
% o valoare in vectorul pwmPeriod
% PwmPeriod = [];
% row_PwmPeriod = 1;
% no_of_rep = 1;
% for j = 1:length(pwmPeriod)
%     if j < length(pwmPeriod)   
%         if pwmPeriod(j) == pwmPeriod(j+1)
%             no_of_rep = no_of_rep +1;
%         else
%             PwmPeriod(row_PwmPeriod,1) = pwmPeriod(j);
%             PwmPeriod(row_PwmPeriod,2) = no_of_rep;
%             no_of_rep = 1;
%             row_PwmPeriod = row_PwmPeriod +1;
%         end
%     else
%         if no_of_rep > 0
%             PwmPeriod(row_PwmPeriod,1) = pwmPeriod(j);
%             PwmPeriod(row_PwmPeriod,2) = no_of_rep;
%             no_of_rep = 1;
%             row_PwmPeriod = row_PwmPeriod +1;
%         end
%     end     
% end

% sum = 0
% % Facem verificari
% for k = 1:length(PwmPeriod)
%     sum = sum + PwmPeriod(k,2);
% end

csvwrite('valoriPWM.txt',new_pwmPeriod_mat{24}');
