#ifndef AUTOMATA_H
#define AUTOMATA_H

#define set_bit(bits, nth) ((bits) |= 1<<(nth))
#define clear_bit(bits,nth) ((bits) &= ~(1<<(nth)))
#define get_bit(bits,nth) ((bits)&1<<(nth) ? 1 : 0)

enum stats {NOT_STARTED, JUST_STARTED, GOING_FWD, ON_EDGE, STOPPED, FINISHED};

void readSwitch(byte switch_id);
void printBits(byte bits);
void printAutomata();
byte genEClosure(byte state);
void genTransition(byte state, byte symbol);
void activateLEDs(byte bits, const byte *addrs);
void processInput(int input);
void showResult();

#endif