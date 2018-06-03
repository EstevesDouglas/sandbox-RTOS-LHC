FRDM-KL25Z
FREESCALE BOARD

02/03/2017
LHC Oficina de IoT 
RTOS aula 1

criando seu RTOS do zero.
utilização do SO windows com o knetis design studio da NXP


RTOS

ponteiro generico maior ponteiro do programa(pilha).

alocar endereço suficiente para o ponteiro.

TCB unidade minima de um sistema operacional.
task,

ulipekernelfindhighprio
schedule do kernel;

implementar 
task resume
task suspende com base com o que está comentado.

task b de menor prioridade chama o resumo
maior prioridade de suspende.

usar task create



09/03/2017

TASK - 

alimentar TCB.


Usar paralel list (

* utilização de bitmap list
grupo de prioridade 

* Criar uma Task

suspender task usar prioridade no mapbit.

função Yield ( schaduler).

ulinekernelTaskYield(); //vai fazer a troca de contexto.

O(n) 
nlog de n.   ele perder realtime no chamada de delay


fechado task.

Exercicio.
proximo objeto de estudo.
 - OsSem.h semafaro.
-- uma ideia de como ele funciona o arquivo função semtake, semgive, semcreate



ISR (a)
ISR (b) - task resumo

(3)TASK a
(4) TASK b

while ( ctrl & bit ).
 - 


pilha nci 
 - pilha base para pagamento nfc

hardfault 
	

treat init.
* inicia todas as outras e roda como treat
(linux faz o mesmo) init some todas as outras.

objeto 

semaforo

struc sema{
 init limit;
 int cnt;
 void * task write;
};

(dar recurso decrementar o recurso disponivel).

Os.config.h





FSR - 
 =  postin
 = preinc
 = postdec




Mutex
OsMutex.c
OsMutex.h

TaskA
TaskB
TAskC

monitonicrate
edf


mutex semafaro binario.
 - ele tenta resolver a questão de prioridade.
 - 

Algoritmo
priority celling

[hiperperiodo]
se repete

homework

descomentar mutex.c. mutexh
criar duas treats

criar recuros compartilhado
buffer na ram
emular um printf


proteger esse buffer usando mutex.


para casa numero
implementar função mutex com priorety celilling.
take e o give
testar com o para casa 1.



Queues/mailbox

void ** ptr;

//ponteiro de ponteiro, apontas apenas para o valor e perder endereço.
ldr r0 , 
rtos flag

ulipe flagpen ( f 0x07, 0)

pend_all

pend_any

//suber as flags que vc precisa
ulipeflagspost(  f , 0,3 
               grupo de flags e flags
