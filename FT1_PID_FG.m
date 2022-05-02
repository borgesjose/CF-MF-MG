%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Fuzzy PID controllers for the Phase                %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 01/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% 1 - Tratando o processo:

% Nesta etapa o processo � discretizado:
% Sendo:

    p1 = (1/2.5);
    p2 = (1/3.75);

    k = 2*p1*p2;
    
    Tc=0.2;
    Tamostra = Tc;
    
% Discretizamos o processo utilizando um segurador de ordem zero:

    s = tf('s');

    ft = k/((s+p1)*(s+p2))

    ftz = c2d(ft,Tc,'zoh')

%% 2 - Aplicando o rele:
% Agora � o momento de aplicar o rel� a planta: (rele com histerese)

    n = 200; % Numero de pontos de an�lise

    eps = 0.2; 
    d = 0.5;

    nptos = 1000

% Chama a fun��o rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplica��o do rel�: 

    [yr,ur] = rele_h(n,Tc,d,eps,[p1,p2],k); 

%     figure;
%     grid;
%     plot(yr,'c-');
%     hold on;
%     plot(ur);

%% 3 Identificar os parametros a partir do experimento com rel�:

    [gw,w,arm,Kp]=Identificar(n, d, eps,Tc,yr,ur);

    Ku = -1/gw;
    Tu = (2*pi)/w;
    %Tu = (2*180)/w;

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
    
%% 3.1 teste modelo:
 a = 1/0.2133;
 b = 0.6667/0.2133;
 c = 0.1067/0.2133;
%% Defini��es do controlador AT-PID-FG: 

    Am = 1;

    Am_min = 2; 
    Am_max = 5;
    Theta_m_min = 45;
    Theta_m_max = 72;
    
    %Theta_m = (180/2)*(1-(1/Am));

%% Sintonizanodo o PID:

    K = (pi/(2*Am*L))*[b;c;a];
    Kc = K(1);
    Ki = K(2);
    Kd = K(3);
    
%% Aplicando o controlador - OLD version
for i=1:nptos,
    if (i<=nptos/2)  ref(i)=1; end;
    if (i>nptos/2)   ref(i) = 2; end;
end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

rlevel = 0.05;
ruido = rlevel*rand(1,nptos);

for i=5:nptos,

P1(i) = p1+rlevel*rand; % Aplicando ruido na modelagem
P2(i) = p2+ruido(i);  % Aplicando ruido na modelagem
k = 2*P1(i)*P2(i); 
    
[c0,c1,c2,r0,r1,r2] = discretiza_zoh(P1(i),P2(i),k,Tc); %chama a fun��o que discretiza o processo utilizano um ZOH;

     if (i==550),r1 = - 1.84;r2 = 0.9109;  end % Ruptura no modelo
     
     y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4); % equa��o da diferen�a do processo
     
     erro(i)=ref(i)-y(i); %Erro
     r(i)=(erro(i)-erro(i-1));%Erro rate
     
            Am(i) = FT1_controler(erro(i),r(i),L);
      
            Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
            %Ami = 1;
      %Controlador:

%             alpha = (Kc)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
%             beta = -(Kc)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
%             gama = (Kc)*(Td)/Tamostra;

            Kci(i) = Kc/Ami;
            Kdi(i) = Kd/Ami;
            Kii(i) = Ki/Ami;

      % new version
            alpha = Kci(i)+ Kdi(i)/Tamostra + (Kii(i)*Tamostra)/2;
            beta = -Kci(i) - 2*(Kdi(i)/Tamostra)+(Kii(i)*Tamostra)/2;
            gama = Kdi(i)/Tamostra;


            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
      
       tempo(i)=i*Tamostra;
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 
 
      ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
     
%plotar seinal de saida e  de controle:    
figure;
grid;
plot(tempo,y,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['FT1-PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
%%
% %plotar P1 e P2
% figure;
% grid;
% plot(tempo,P1,'g-');
% hold on;
% plot(tempo,P2);
%%
%plotar Kp,Kd,Ki
figure;
grid;
plot(tempo,Kci,'g-');
hold on;
plot(tempo,Kdi);
hold on;
plot(tempo,Kii);
title('FT1-PID-FG: Kp,Ki,Kd')
legend('Kc','Kd','Ki')
%%
figure;
grid;
plot(tempo,Am,'g-');
title('FT1-PID-FG: Am')