import serial
import subprocess
import os
from math import pi, sin, cos

SERIAL_PORT = '/dev/cu.wchusbserial1410'
SERIAL_BPS  = 9600

STATES = 5

print('Estabelecendo conexão com o Arduino...')
board = serial.Serial(SERIAL_PORT, SERIAL_BPS)

def readSwitch():
    return list(map(int, list(board.readline().decode())[:-1]))

transition = [[], [], []]

print('Carregando configuração...')

for symbol in range(3):
    for i in range(STATES):
        transition[symbol].append(readSwitch())

initial_state = readSwitch()
final_states = readSwitch()

useful_states  = [0]*STATES

for symbol in range(3):
    for q in range(STATES):
        for j in range(STATES):
            if transition[symbol][q][j] or transition[symbol][j][q]:
                useful_states[q] = 1
                break

total_states = useful_states.count(1)

alpha = 2.0 * pi / (total_states);
radius = total_states * 120.0 / (2.0 * pi);

print('Gerando arquivo JFLAP...')

with open('generated.jff', 'w') as f:
    _read = ['<read>0</read>', '<read>1</read>', '<read/>']
    f.write('<?xml version="1.0" encoding="UTF-8" standalone="no"?>')
    f.write('<structure>')
    f.write('<type>fa</type>')
    f.write('<automaton>')
    for ind, val in enumerate(useful_states):
        if val:
            f.write('<state id="%d" name="q%d">' % (ind, ind+1))
            f.write('<x>%f</x>' % (cos(pi + ind*alpha)*radius + 1.7*radius))
            f.write('<y>%f</y>' % (sin(pi + ind*alpha)*radius + 1.7*radius))
            if initial_state[ind]:
                f.write('<initial/>')
            if final_states[ind]:
                f.write('<final/>')
            f.write('</state>')

    for symbol in range(3):
        for q in range (STATES):
            for j in range (STATES):
                if transition[symbol][q][j]:
                    f.write('<transition>')
                    f.write(f'<from>%d</from>' % q)
                    f.write(f'<to>%d</to>' % j)
                    f.write(_read[symbol])
                    f.write('</transition>')

    f.write('</automaton>')
    f.write('</structure>')

print('Abrindo arquivo gerado no JFLAP...')
subprocess.Popen(['java', '-jar', 'JFLAP.jar', 'generated.jff'])


