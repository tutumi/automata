#include <Arduino.h>
#include "automata.h"

#define STATES 5 // Número máximo de estados.

// Cada switch é um conjunto de (n switches) individuais que serão ativados simultaneamente.
#define SWITCHES STATES*3+2 // STATES*3 para os switches das tabelas de transição + inicial + finais.

#define MOTOR_SPEED 230 // Duty cycle no pino do motor. (0~255)

// Pinos OUTPUT para os LEDs que representam os estados ativos.
const byte addr_reg[STATES] = {22, 23, 24, 25, 26}; // q[0], ..., q[n]

// Pinos OUTPUT para os LEDs que representam os estados que ficarão ativos após a leitura do símbolo.
const byte addr_aux[STATES] = {32, 33, 34, 35, 36}; // q[0], ..., q[n]

// Pinos OUTPUT que serão utilizados para ativar os switches.
const byte addr_sw_in[SWITCHES] = {
    A0, A1, A2, A3, A4,  // delta(q[0], 0), ..., delta(q[n], 0)
    A5, A6, A7, A8, A9,  // delta(q[0], 1), ..., delta(q[n], 1)
    42, 43, 44, 45, 46,  // delta(q[0], E), ..., delta(q[n], E)
    A10,                 // Estado inicial
    A11                  // Estados de aceitação
};                

// Pinos INPUT (pull-down) para a leitura da saída dos switches.
const byte _sw_out[][STATES] = {
    {2, 3, 4, 5, 6} // bit[0], ..., bit[n]
};

// Associação entre cada switch e seus respectivos pinos de saída.
const byte* addr_sw_out[SWITCHES] = { 
    _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], // delta(q[0], 0), ..., delta(q[n], 0)
    _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], // delta(q[0], 1), ..., delta(q[n], 1)
    _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], _sw_out[0], // delta(q[0], E), ..., delta(q[n], E)
    _sw_out[0],                                                 // Estado inicial
    _sw_out[0]                                                  // Estados de aceitação
};

// Pinos OUTPUT para os LEDs que indicam se a cadeia de entrada foi aceita ou não.
const byte addr_accept = 52;
const byte addr_reject = 53;

// Pinos INPUT_PULLUP do leitor de fita.
const byte addr_tape_sync  =  8; // Trilha de sincronização
const byte addr_tape_data  =  9; // Trilha de dados
const byte addr_tape_end   = 10; // Trilha de fim de papel
const byte addr_tape_next  = 12; // Botão para leitura do próximo símbolo
const byte addr_tape_motor = 11; // Transistor do motor

// Variáveis internas do código.
byte delta[STATES][3];        // Função de transição
byte e_closure[STATES] = {0}; // E-Closure de cada estado
byte transition[STATES][2];   // Transição pré-calculada com a E-Closure.

byte initial_state = 0; // Estado inicial
byte accept_states = 0; // Estados de Aceitação

byte reg = 0; // Registrador principal
byte aux = 0; // Registrador auxiliar

enum stats status = NOT_STARTED; // Estado atual do sistema

void setup() {
    Serial.begin(9600);

    pinMode(addr_tape_sync, INPUT_PULLUP);
    pinMode(addr_tape_data, INPUT_PULLUP);
    pinMode(addr_tape_end, INPUT_PULLUP);
    pinMode(addr_tape_next, INPUT_PULLUP);
    pinMode(addr_tape_motor, OUTPUT);
    
    pinMode(addr_accept, OUTPUT);
    pinMode(addr_reject, OUTPUT);

    for (byte i = 0; i < SWITCHES; ++i)
        pinMode(addr_sw_in[i], OUTPUT);

    for (byte i = 0; i < STATES; ++i) {
        pinMode(addr_reg[i], OUTPUT);
        pinMode(addr_aux[i], OUTPUT);
    }

    for (byte i = 0; i < sizeof(addr_sw_out)/STATES; ++i)
        for (byte j = 0; j < STATES; ++j)
            pinMode(addr_sw_out[i][j], INPUT);

    for (byte i = 0; i < STATES*3; ++i)
        readSwitch(i, delta[i%STATES][i/STATES]);

    readSwitch(SWITCHES-2, initial_state);
    readSwitch(SWITCHES-1, accept_states);

    printAutomata();

    for (byte state = 0; state < STATES; ++state)
        genEClosure(state);

    for (byte symbol = 0; symbol < 2; ++symbol)
        for (byte state = 0; state < STATES; ++state)
            genTransition(state, symbol);

    aux = e_closure[(byte)(log(initial_state)/log(2))];

    reg = initial_state;
    activateLEDs(reg, addr_reg);
}

void loop() {
    switch (status) {
        case STOPPED:
            if (!digitalRead(addr_tape_next)) {
                analogWrite(addr_tape_motor, MOTOR_SPEED);
                reg = aux;
                activateLEDs(reg, addr_reg);
                status = GOING_FWD;
            }
            break;
        case GOING_FWD:
            if (digitalRead(addr_tape_sync))
                status = ON_EDGE;
            break;
        case ON_EDGE:
            if (!digitalRead(addr_tape_end)) {
                showResult();
                delay(200);
                digitalWrite(addr_tape_motor, LOW);
                status = FINISHED;
            } else if (!digitalRead(addr_tape_sync)) {
                digitalWrite(addr_tape_motor, LOW);
                processInput((byte)(digitalRead(addr_tape_data) ^ 1));
                status = STOPPED;
            }
            break;
        case NOT_STARTED:
            if (digitalRead(addr_tape_sync)) {
                analogWrite(addr_tape_motor, MOTOR_SPEED);
                status = JUST_STARTED;
            }
            break;
        case JUST_STARTED:
            if (!digitalRead(addr_tape_sync)) {
                digitalWrite(addr_tape_motor, LOW);
                status = STOPPED;
            }
            break;
    }
}

void readSwitch(byte switch_id, byte& bits) {
    digitalWrite(addr_sw_in[switch_id], HIGH);

    for (byte state = 0; state < STATES; ++state)
        digitalRead(addr_sw_out[switch_id][state]) ? set_bit(bits, state) : clear_bit(bits, state);

    digitalWrite(addr_sw_in[switch_id], LOW);
}

void printBits(byte bits) {
    for (byte i = 0; i < STATES; ++i)
        Serial.print(get_bit(bits, i));
    Serial.print("\n");
}

void printAutomata() {
    for (byte symbol = 0; symbol < 3; ++symbol)
        for (byte state = 0; state < STATES; ++state)
            printBits(delta[state][symbol]);

    printBits(initial_state);
    printBits(accept_states);
}

byte genEClosure(byte state) {
    if (!e_closure[state]) {
        set_bit(e_closure[state], state);
        for (byte i = 0; i < STATES; ++i)
            if (get_bit(delta[state][2], i) && !get_bit(e_closure[state], i))
                e_closure[state] |= genEClosure(i);
    }
    return e_closure[state];
}

void genTransition(byte state, byte symbol) {
    for (byte i = 0; i < STATES; ++i)
        if (get_bit(delta[state][symbol], i))
            transition[state][symbol] |= e_closure[i];
}

void activateLEDs(byte bits, const byte *addrs) {
    for (byte i = 0; i < STATES; ++i)
        digitalWrite(addrs[i], get_bit(bits, i));
}

void processInput(byte input) {
    aux = 0;
    
    for (byte state = 0; state < STATES; ++state)
        if (get_bit(reg, state))
            aux |= transition[state][input];

    activateLEDs(aux, addr_aux);   
}

void showResult() {
    if (reg & accept_states)
        digitalWrite(addr_accept, HIGH);
    else
        digitalWrite(addr_reject, HIGH);
}