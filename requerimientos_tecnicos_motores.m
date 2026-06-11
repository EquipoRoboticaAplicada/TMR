%% CALCULO DE ESPECIFICACIONES DE MOTOR PARA ROVER 6 RUEDAS
% Rover tipo rocker-bogie
% Calcula torque, RPM, reduccion, potencia, corriente y torque stall recomendado

clear;
clc;

%% =======================
%  1. DATOS DEL ROVER
%  =======================

m = 23;                  % masa total del rover [kg]
g = 9.81;                % gravedad [m/s^2]

diametro_rueda = 0.17;   % diametro de rueda [m]
r = diametro_rueda/2;    % radio de rueda [m]

theta_deg = 45;          % pendiente maxima [grados]
theta = deg2rad(theta_deg);

n_ruedas = 6;            % numero total de ruedas motrices
n_ruedas_efectivas = n_ruedas;  % ruedas efectivas para diseño conservador

Crr = 0.20;              % coeficiente de resistencia al rodamiento

% Crr significa coeficiente de resistencia al rodamiento. 
% Representa qué tanta fuerza adicional necesita el rover para avanzar 
% debido al contacto entre la rueda y el suelo.

% No es lo mismo avanzar sobre concreto que sobre arena, tierra, grava 
% o pasto. En piso duro, el valor puede ser bajo. En terreno suelto, la 
% rueda se hunde, deforma el suelo y necesita más esfuerzo para avanzar.

% Mientras mayor sea Crr, mayor será el torque requerido por los motores.

a = 0.20;                % aceleracion deseada [m/s^2]

FS = 2.5;                % factor de seguridad recomendado: 2 a 3

%% =======================
%  2. DATOS DE DESEMPENO
%  =======================

v_objetivo = 0.30;       % velocidad lineal deseada del rover [m/s]

rpm_motor_nominal = 5000; % velocidad aproximada del motor antes de reduccion [RPM]

%% Ficha técnica del motor JGB37-3530-56.25K
% Velocidad sin carga (12V @ 0.07A): 89RPM
% Velocidad nominal / con carga de trabajo (12V @ 0.48A): 67RPM
% Reducción: 56.25

%% Por lo tanto...
% velocidad sin carga antes de reducción: 5006RPM
% Velocidad nominal antes de reducción: 3768RPM

%% =======================
%  3. EFICIENCIAS
%  =======================

eta_caja = 0.75;         % eficiencia de caja reductora planetaria
eta_motor = 0.80;        % eficiencia aproximada del motor DC con escobillas
eta_driver = 0.95;       % eficiencia aproximada del driver

eta_total = eta_caja * eta_motor * eta_driver;

V_sistema = 14.8;          % voltaje del sistema [V]

%% =======================
%  4. FUERZAS DEL ROVER
%  =======================

F_pendiente = m * g * sin(theta);

F_rodamiento = Crr * m * g * cos(theta);

F_aceleracion = m * a;

F_total = F_pendiente + F_rodamiento + F_aceleracion;

F_por_rueda = F_total / n_ruedas_efectivas;

%% =======================
%  5. TORQUE DE SALIDA POR RUEDA
%  =======================

T_salida_sin_FS = F_por_rueda * r;

T_salida_con_FS = T_salida_sin_FS * FS;

% Conversion a kgf*cm
Nm_a_kgfcm = 10.1972;

T_salida_sin_FS_kgfcm = T_salida_sin_FS * Nm_a_kgfcm;
T_salida_con_FS_kgfcm = T_salida_con_FS * Nm_a_kgfcm;

%% =======================
%  6. VELOCIDAD DE SALIDA REQUERIDA
%  =======================

rpm_rueda = (v_objetivo / (2*pi*r)) * 60;

omega_rueda = (2*pi*rpm_rueda) / 60; % [rad/s]

%% =======================
%  7. RELACION DE REDUCCION NECESARIA
%  =======================

relacion_reduccion = rpm_motor_nominal / rpm_rueda;

%% =======================
%  8. TORQUE REQUERIDO EN EL EJE DEL MOTOR
%  =======================
% Este valor aplica si se analiza el motor ANTES de la caja reductora.
% Si compras un motorreductor, normalmente debes comparar contra
% T_salida_con_FS, porque ese torque ya es en la salida de la caja.

T_motor_antes_caja = T_salida_con_FS / (relacion_reduccion * eta_caja);

%% =======================
%  9. POTENCIA MECANICA POR MOTOR
%  =======================

P_mecanica_salida = T_salida_con_FS * omega_rueda;

P_electrica_aprox = P_mecanica_salida / eta_total;

P_total_mecanica = P_mecanica_salida * n_ruedas;
P_total_electrica = P_electrica_aprox * n_ruedas;

%% =======================
%  10. CORRIENTE REQUERIDA POR MOTOR
%  =======================

I_nominal_aprox = P_electrica_aprox / V_sistema;

I_total_aprox = I_nominal_aprox * n_ruedas;

%% =======================
%  11. TORQUE STALL RECOMENDADO
%  =======================
% Practica de diseño:
% El torque de trabajo no debe estar cerca del stall.
% Se recomienda que el torque de trabajo sea entre 25% y 50%
% del torque stall del motorreductor.

T_stall_min_50 = T_salida_con_FS / 0.50;
T_stall_min_25 = T_salida_con_FS / 0.25;

T_stall_min_50_kgfcm = T_stall_min_50 * Nm_a_kgfcm;
T_stall_min_25_kgfcm = T_stall_min_25 * Nm_a_kgfcm;

%% =======================
%  12. RESULTADOS
%  =======================

fprintf('\n============================================\n');
fprintf(' ESPECIFICACIONES CALCULADAS PARA EL MOTOR\n');
fprintf('============================================\n\n');

fprintf('Datos del rover:\n');
fprintf('Masa total: %.2f kg\n', m);
fprintf('Diametro de rueda: %.3f m\n', diametro_rueda);
fprintf('Radio de rueda: %.3f m\n', r);
fprintf('Pendiente maxima: %.1f grados\n', theta_deg);
fprintf('Velocidad objetivo: %.2f m/s\n', v_objetivo);
fprintf('Ruedas totales: %d\n', n_ruedas);
fprintf('Ruedas efectivas usadas para calculo: %d\n\n', n_ruedas_efectivas);

fprintf('Fuerzas:\n');
fprintf('Fuerza por pendiente: %.2f N\n', F_pendiente);
fprintf('Fuerza por rodamiento: %.2f N\n', F_rodamiento);
fprintf('Fuerza por aceleracion: %.2f N\n', F_aceleracion);
fprintf('Fuerza total requerida: %.2f N\n', F_total);
fprintf('Fuerza por rueda efectiva: %.2f N\n\n', F_por_rueda);

fprintf('1. Torque de salida requerido por rueda:\n');
fprintf('Torque sin FS: %.2f N*m  |  %.2f kgf*cm\n', ...
    T_salida_sin_FS, T_salida_sin_FS_kgfcm);
fprintf('Torque con FS %.1f: %.2f N*m  |  %.2f kgf*cm\n\n', ...
    FS, T_salida_con_FS, T_salida_con_FS_kgfcm);

fprintf('2. Velocidad de salida requerida:\n');
fprintf('RPM de salida en la rueda: %.2f RPM\n', rpm_rueda);
fprintf('Velocidad angular: %.2f rad/s\n\n', omega_rueda);

fprintf('3. Relacion de reduccion necesaria:\n');
fprintf('RPM motor nominal: %.0f RPM\n', rpm_motor_nominal);
fprintf('Relacion de reduccion recomendada: %.1f : 1\n\n', relacion_reduccion);

fprintf('4. Potencia mecanica por motor:\n');
fprintf('Potencia mecanica de salida por motor: %.2f W\n', P_mecanica_salida);
fprintf('Potencia electrica aproximada por motor: %.2f W\n', P_electrica_aprox);
fprintf('Potencia mecanica total 6 motores: %.2f W\n', P_total_mecanica);
fprintf('Potencia electrica total aproximada: %.2f W\n\n', P_total_electrica);

fprintf('5. Corriente requerida por motor:\n');
fprintf('Corriente nominal aproximada por motor a %.0f V: %.2f A\n', ...
    V_sistema, I_nominal_aprox);
fprintf('Corriente total aproximada para %d motores: %.2f A\n\n', ...
    n_ruedas, I_total_aprox);

fprintf('6. Torque stall recomendado:\n');
fprintf('Torque stall minimo si trabajo = 50%% del stall: %.2f N*m | %.2f kgf*cm\n', ...
    T_stall_min_50, T_stall_min_50_kgfcm);
fprintf('Torque stall recomendado si trabajo = 25%% del stall: %.2f N*m | %.2f kgf*cm\n\n', ...
    T_stall_min_25, T_stall_min_25_kgfcm);

fprintf('Torque requerido en eje del motor antes de caja:\n');
fprintf('T_motor = %.4f N*m antes de reduccion\n\n', T_motor_antes_caja);

%% =======================
%  13. TABLA RESUMEN
%  =======================

Parametro = [
    "Torque salida con FS por rueda";
    "Torque salida con FS por rueda";
    "RPM salida rueda";
    "Relacion de reduccion";
    "Potencia mecanica por motor";
    "Potencia electrica aprox por motor";
    "Corriente nominal aprox por motor";
    "Torque stall minimo";
    "Torque stall recomendado"
];

Valor = [
    T_salida_con_FS;
    T_salida_con_FS_kgfcm;
    rpm_rueda;
    relacion_reduccion;
    P_mecanica_salida;
    P_electrica_aprox;
    I_nominal_aprox;
    T_stall_min_50;
    T_stall_min_25
];

Unidad = [
    "N*m";
    "kgf*cm";
    "RPM";
    ":1";
    "W";
    "W";
    "A";
    "N*m";
    "N*m"
];

tabla_resultados = table(Parametro, Valor, Unidad);

disp('Tabla resumen de especificaciones:');
disp(tabla_resultados);

%% =======================
%  14. VALIDACION OPCIONAL DE UN MOTOR CANDIDATO
%  =======================
% Cambia estos valores si ya tienes un motor en mente.
% Si no quieres validar un motor, deja los valores como NaN.

% T_cont_candidato = NaN;      % torque continuo de salida del motorreductor [N*m]
% T_stall_candidato = NaN;     % torque stall de salida del motorreductor [N*m]
% rpm_salida_candidato = 67;  % RPM de salida del motorreductor [RPM]
% I_nom_candidato = NaN;       % corriente nominal del motor [A]
% I_stall_candidato = NaN;     % corriente stall del motor [A]

% Datos del motor candidato segun ficha tecnica a 12 V

kgfcm_a_Nm = 0.0980665;  % conversion de kgf*cm a N*m

T_cont_candidato = 6.5 * kgfcm_a_Nm;      % torque aprox. a maxima eficiencia [N*m]
T_stall_candidato = 49 * kgfcm_a_Nm;      % torque de bloqueo/stall [N*m]
rpm_salida_candidato = 58;                % RPM a maxima eficiencia [RPM]
I_nom_candidato = 0.72;                   % corriente a maxima eficiencia [A]
I_stall_candidato = 5.5;                  % corriente de bloqueo/stall [A]

fprintf('\n============================================\n');
fprintf(' VALIDACION DE MOTOR CANDIDATO\n');
fprintf('============================================\n');

if isnan(T_cont_candidato)
    fprintf('No se ingreso motor candidato.\n');
    fprintf('Para validar uno, llena T_cont_candidato, T_stall_candidato,\n');
    fprintf('rpm_salida_candidato, I_nom_candidato e I_stall_candidato.\n');
else
    if T_cont_candidato >= T_salida_con_FS
        fprintf('Torque continuo: CUMPLE (%.2f N*m)\n', T_cont_candidato);
    else
        fprintf('Torque continuo: NO CUMPLE (%.2f N*m)\n', T_cont_candidato);
    end

    if T_stall_candidato >= T_stall_min_50
        fprintf('Torque stall: CUMPLE MINIMO (%.2f N*m)\n', T_stall_candidato);
    else
        fprintf('Torque stall: NO CUMPLE (%.2f N*m)\n', T_stall_candidato);
    end

    if rpm_salida_candidato >= rpm_rueda
        fprintf('RPM salida: CUMPLE (%.2f RPM)\n', rpm_salida_candidato);
    else
        fprintf('RPM salida: NO CUMPLE (%.2f RPM)\n', rpm_salida_candidato);
    end

    if I_nom_candidato >= I_nominal_aprox
        fprintf('Corriente nominal del motor: coherente con demanda\n');
    else
        fprintf('Corriente nominal del motor: revisar, puede quedar corto\n');
    end

    fprintf('Corriente stall del candidato: %.2f A\n', I_stall_candidato);
    fprintf('Selecciona un driver que soporte corriente nominal y picos.\n');
end