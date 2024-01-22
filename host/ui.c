#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/types.h>


#include <pthread.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define DEVICE ("/dev/ttyACM0")
#define UART_BAUDRATE (B115200)
#define BUFF_SIZE (100)
#define DEC_DELAY (20)

// Funcion para detectar si se presiono una tecla
int kbhit(void) {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); // STDIN_FILENO es el descriptor de archivo estándar para la entrada estándar (teclado)
    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

// Funcion para cambiar los flag de la configuracion del puerto serie
void clearFlag(tcflag_t *flags, tcflag_t flag, const char *flagName) {
    if (*flags & flag) {
        printf("%s estaba activado y ha sido desactivado.\n", flagName);
        *flags &= ~flag; // Desactivar el flag solo si está activado
    }
}

// Mutex para evitar condiciones de carrera en la escritura al puerto serie
pthread_mutex_t serialMutex = PTHREAD_MUTEX_INITIALIZER;

// Mutex para avisar a la tarea principal que se ha leido un caracter del teclado
pthread_mutex_t tecladoMutex = PTHREAD_MUTEX_INITIALIZER;

// Mutex para indicar a la tarea teclado que proceda a leer un caracter del teclado
pthread_mutex_t empezarLeer = PTHREAD_MUTEX_INITIALIZER;

// Mutex para evitar condiciones de carrera en la variable terminarPrograma
pthread_mutex_t terminarMutex = PTHREAD_MUTEX_INITIALIZER;

// Variable compartida para indicar si el programa debe terminar
int terminarPrograma = 0;

// Variable compartida para almacenar el caracter leido del teclado
char teclaLeida = 0;

// Variable para guardar el descriptor de archivo del puerto serie
int serial_port = -1;

// Variables para guardar configuracion de puerto serie
struct termios tty;
struct termios tty_old;

// Función principal que simula la tarea principal
void* tareaPrincipal(void* arg) {
    int estado = 0;
    int delay_dsec = DEC_DELAY;
    int baudrate = UART_BAUDRATE;
    char device [30] = DEVICE;
    char option = '0';
    char tecla = '0';
    struct termios oldt, newt;
    int liberar = 0;
    int local_terminarPrograma = 0;
    
    // Guarda la configuración actual de la terminal
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    pthread_mutex_lock(&empezarLeer);
    pthread_mutex_lock(&serialMutex);
    printf("Tarea principal ejecutándose...\n");

    // Bucle principal
    pthread_mutex_lock(&terminarMutex);
    local_terminarPrograma = terminarPrograma;
    pthread_mutex_unlock(&terminarMutex);
    while (!local_terminarPrograma) {
        // Coloca aquí la lógica de la tarea principal
        if(estado == 0){
            printf("Estado: Desconectado\n");
            printf("Menu principal\n");
            printf("1. Conectar\n");
            printf("2. Configurar\n");
            printf("q. Salir\n");
            printf("Opcion: ");
            scanf(" %c", &tecla);
            switch(tecla){
                case '1':
                    // Conectado
                    estado = 1;
                    // Desactiva la línea de entrada y la impresión de entrada
                    newt.c_lflag &= ~(ICANON | ECHO);
                    // Establece la nueva configuración inmediatamente
                    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
                    // Configurar puerto serie
                    // Abrir puerto serie
                    serial_port = open(device, O_RDWR | O_NOCTTY | O_SYNC);
                    // Ver si no existe configuracion, o que no existe comunicacion serie 
                    if(tcgetattr(serial_port, &tty) != 0) {
                        printf("Error %i de tcgetattr: %s\n", errno, strerror(errno));
                        pthread_mutex_lock(&terminarMutex);
                        terminarPrograma = 1;
                        pthread_mutex_unlock(&terminarMutex);
                        break;
                    }
                    // Establecer configuracion de baud rate
                    cfsetispeed(&tty, baudrate);
                    cfsetospeed(&tty, baudrate);
                    // Cambiar configuracion a nuestras necesidades
                    // tty.c_cflag &= ~PARENB; // Eliminar bit de paridad
                    // tty.c_cflag &= ~CSTOPB; // Eliminar bit de stop, solo usar 1
                    // tty.c_cflag &= ~CSIZE; // Eliminar bit de tamaño de caracter
                    // tty.c_cflag |= CS8; // Usar 8 bits por caracter
                    // tty.c_cflag &= ~CRTSCTS; // Desactivar control de flujo
                    // tty.c_cflag |= CREAD | CLOCAL; // Activar lectura y ignorar control de linea

                    // tty.c_lflag &= ~ICANON; // Desactivar modo canonico
                    // tty.c_lflag &= ~ECHO; // Desactivar eco
                    // tty.c_lflag &= ~ECHOE; // Desactivar eco de caracter de borrado
                    // tty.c_lflag &= ~ECHONL; // Desactivar eco nueva linea
                    // tty.c_lflag &= ~ISIG; // Desactivar señales
                    // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Desactivar control de flujo
                    // tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Desactivar control de flujo

                    // tty.c_oflag &= ~OPOST; // Desactivar procesamiento de salida
                    // tty.c_oflag &= ~ONLCR; // Desactivar conversion de nueva linea
                    // tty.c_cc[VTIME] = 0; // Bloquear 0 segundos
                    // tty.c_cc[VMIN] = 12; // Leer 0 bytes minimo

                    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
                    // disable IGNBRK for mismatched speed tests; otherwise receive break
                    // as \000 chars
                    tty.c_iflag &= ~IGNBRK;         // disable break processing
                    tty.c_lflag = 0;                // no signaling chars, no echo,
                                                    // no canonical processing
                    tty.c_oflag = 0;                // no remapping, no delays
                    tty.c_cc[VMIN]  = 0;            // read doesn't block
                    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

                    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

                    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                                    // enable reading
                    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
                    tty.c_cflag &= ~CSTOPB;
                    tty.c_cflag &= ~CRTSCTS;

                    // Guardar configuracion de puerto serie
                    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                        printf("Error %i de tcsetattr: %s\n", errno, strerror(errno));
                        pthread_mutex_lock(&terminarMutex);
                        terminarPrograma = 1;
                        pthread_mutex_unlock(&terminarMutex);
                        break;
                    }
                    
                    if (serial_port < 0) {
                        printf("Error %i al abrir el puerto serie: %s\n", errno, strerror(errno));
                        pthread_mutex_lock(&terminarMutex);
                        terminarPrograma = 1;
                        pthread_mutex_unlock(&terminarMutex);
                        break;
                    }
                    pthread_mutex_unlock(&empezarLeer);
                    pthread_mutex_unlock(&serialMutex);
                    liberar = 1;
                    break;
                case '2':
                    // Configurar
                    printf("Configurando...\n");
                    // Imprimir configuracion actual
                    printf("Configuracion actual:\n");
                    printf("Dispositivo: %s\n", device);
                    // Imprimir el baudrate en funcion de la macro
                    switch(baudrate){
                        case B0:
                            printf("Baudrate: B0\n");
                            break;
                        case B50:
                            printf("Baudrate: B50\n");
                            break;
                        case B75:
                            printf("Baudrate: B75\n");
                            break;
                        case B110:
                            printf("Baudrate: B110\n");
                            break;
                        case B134:
                            printf("Baudrate: B134\n");
                            break;
                        case B150:
                            printf("Baudrate: B150\n");
                            break;
                        case B200:
                            printf("Baudrate: B200\n");
                            break;
                        case B300:
                            printf("Baudrate: B300\n");
                            break;
                        case B600:
                            printf("Baudrate: B600\n");
                            break;
                        case B1200:
                            printf("Baudrate: B1200\n");
                            break;
                        case B1800:
                            printf("Baudrate: B1800\n");
                            break;
                        case B2400:
                            printf("Baudrate: B2400\n");
                            break;
                        case B4800:
                            printf("Baudrate: B4800\n");
                            break;
                        case B9600:
                            printf("Baudrate: B9600\n");
                            break;
                        case B19200:
                            printf("Baudrate: B19200\n");
                            break;
                        case B38400:
                            printf("Baudrate: B38400\n");
                            break;
                        case B57600:
                            printf("Baudrate: B57600\n");
                            break;
                        case B115200:
                            printf("Baudrate: B115200\n");
                            break;
                        case B230400:
                            printf("Baudrate: B230400\n");
                            break;
                        case B460800:
                            printf("Baudrate: B460800\n");
                            break;
                        case B500000:
                            printf("Baudrate: B500000\n");
                            break;
                        case B576000:
                            printf("Baudrate: B576000\n");
                            break;
                        case B921600:
                            printf("Baudrate: B921600\n");
                            break;
                        case B1000000:
                            printf("Baudrate: B1000000\n");
                            break;
                        case B1152000:
                            printf("Baudrate: B1152000\n");
                            break;
                        case B1500000:
                            printf("Baudrate: B1500000\n");
                            break;
                        case B2000000:
                            printf("Baudrate: B2000000\n");
                            break;
                        case B2500000:
                            printf("Baudrate: B2500000\n");
                            break;
                        default:
                            printf("Baudrate: %i\n", baudrate);
                            break;
                    }
                    printf("Delay (decimas de segundo): %d\n", delay_dsec);
                    // Preguntar si se quiere cambiar
                    printf("Cambiar configuracion? (y/n): ");
                    scanf(" %c", &option);
                    if(option == 'n')
                        break;
                    // Leer configuracion
                    // Variables para guardar configuracion
                    char new_device[30] = "";
                    char new_baudrate [30] = "";
                    int new_baudrate_int = -1;
                    int new_delay_dsec = -1;
                    // Si se lee -1, no se cambia
                    printf("Si no se quiere cambiar, escribir NULL o -1\n");
                    printf("Dispositivo: ");
                    scanf("%s", new_device);
                    printf("Baudrate: ");
                    scanf("%s", new_baudrate);
                    // Comprobar que el baudrate es correcto
                    if(strcmp(new_baudrate, "B0") == 0)
                        new_baudrate_int = B0;
                    else if(strcmp(new_baudrate, "B50") == 0)
                        new_baudrate_int = B50;
                    else if(strcmp(new_baudrate, "B75") == 0)
                        new_baudrate_int = B75;
                    else if(strcmp(new_baudrate, "B110") == 0)
                        new_baudrate_int = B110;
                    else if(strcmp(new_baudrate, "B134") == 0)
                        new_baudrate_int = B134;
                    else if(strcmp(new_baudrate, "B150") == 0)
                        new_baudrate_int = B150;
                    else if(strcmp(new_baudrate, "B200") == 0)
                        new_baudrate_int = B200;
                    else if(strcmp(new_baudrate, "B300") == 0)
                        new_baudrate_int = B300;
                    else if(strcmp(new_baudrate, "B600") == 0)
                        new_baudrate_int = B600;
                    else if(strcmp(new_baudrate, "B1200") == 0)
                        new_baudrate_int = B1200;
                    else if(strcmp(new_baudrate, "B1800") == 0)
                        new_baudrate_int = B1800;
                    else if(strcmp(new_baudrate, "B2400") == 0)
                        new_baudrate_int = B2400;
                    else if(strcmp(new_baudrate, "B4800") == 0)
                        new_baudrate_int = B4800;
                    else if(strcmp(new_baudrate, "B9600") == 0)
                        new_baudrate_int = B9600;
                    else if(strcmp(new_baudrate, "B19200") == 0)
                        new_baudrate_int = B19200;
                    else if(strcmp(new_baudrate, "B38400") == 0)
                        new_baudrate_int = B38400;
                    else if(strcmp(new_baudrate, "B57600") == 0)
                        new_baudrate_int = B57600;
                    else if(strcmp(new_baudrate, "B115200") == 0)
                        new_baudrate_int = B115200;
                    else if(strcmp(new_baudrate, "B230400") == 0)
                        new_baudrate_int = B230400;
                    else if(strcmp(new_baudrate, "B460800") == 0)
                        new_baudrate_int = B460800;
                    else if(strcmp(new_baudrate, "B500000") == 0)
                        new_baudrate_int = B500000;
                    else if(strcmp(new_baudrate, "B576000") == 0)
                        new_baudrate_int = B576000;
                    else if(strcmp(new_baudrate, "B921600") == 0)
                        new_baudrate_int = B921600;
                    else if(strcmp(new_baudrate, "B1000000") == 0)
                        new_baudrate_int = B1000000;
                    else if(strcmp(new_baudrate, "B1152000") == 0)
                        new_baudrate_int = B1152000;
                    else if(strcmp(new_baudrate, "B1500000") == 0)
                        new_baudrate_int = B1500000;
                    else if(strcmp(new_baudrate, "B2000000") == 0)
                        new_baudrate_int = B2000000;
                    else if(strcmp(new_baudrate, "B2500000") == 0)
                        new_baudrate_int = B2500000;
                    else
                        new_baudrate_int = -1;
                    printf("Delay (entre 12 y 255): ");
                    scanf("%i", &new_delay_dsec);
                    // Cambiar configuracion
                    printf("Cambiando configuracion...\n");
                    printf("%s\n", new_device);
                    if(strcmp(new_device, "NULL") != 0 && strcmp(new_device, "-1") != 0 && strcmp(new_device, "") != 0 && strcmp(new_device, " ") != 0)
                        //device = new_device;
                        strcpy(device, new_device);
                    if(new_baudrate_int != -1)
                        baudrate = new_baudrate_int;
                    if(new_delay_dsec != -1 && new_delay_dsec >= 12 && new_delay_dsec <= 255)
                        delay_dsec = new_delay_dsec;
                    // Imprimir configuracion nueva
                    printf("Configuracion nueva:\n");
                    printf("Dispositivo: %s \n", device);
                    printf("Baudrate: %i \n", baudrate);
                    printf("Delay: %i \n", delay_dsec);
                    break;
                case 'q':
                    // Salir
                    pthread_mutex_lock(&terminarMutex);
                    terminarPrograma = 1;
                    pthread_mutex_unlock(&terminarMutex);
                    break;
                default:
                    break;
            }
        }else{
            // Coloca aquí la lógica para leer el caracter leido del teclado
            pthread_mutex_lock(&tecladoMutex);
            tecla = teclaLeida;
            pthread_mutex_unlock(&tecladoMutex);
            if(tecla != -1){
                printf("Se ha leido la tecla %c\n", tecla);
            }
            if (tecla == 'q' || tecla == 'Q') {
                pthread_mutex_lock(&terminarMutex);
                terminarPrograma = 1;
                pthread_mutex_unlock(&terminarMutex);
            }
        }
        
        // Esperar 1 segundos
        usleep(1000000);
        pthread_mutex_lock(&terminarMutex);
        local_terminarPrograma = terminarPrograma;
        pthread_mutex_unlock(&terminarMutex);
    }
    if(!liberar){
        pthread_mutex_unlock(&empezarLeer);
        pthread_mutex_unlock(&serialMutex);
    }
    printf("Tarea principal terminada.\n");
    // Restaurar configuracion de terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return NULL;
}

// Función para la tarea que lee teclas del teclado
void* tareaTeclado(void* arg) {
    char tecla = 0;
    int local_terminarPrograma = 0;
    pthread_mutex_lock(&empezarLeer);
    pthread_mutex_unlock(&empezarLeer);
    pthread_mutex_lock(&terminarMutex);
    local_terminarPrograma = terminarPrograma;
    pthread_mutex_unlock(&terminarMutex);
    while (!local_terminarPrograma) {
        if(kbhit()) tecla = getchar();  // Obtener la tecla
        else tecla = -1;

        pthread_mutex_lock(&tecladoMutex);
        teclaLeida = tecla;
        pthread_mutex_unlock(&tecladoMutex);
        if (tecla == 'q' || tecla == 'Q') {
            break;
        }
        usleep(200000);  // Espera 0.2 segundos
        pthread_mutex_lock(&terminarMutex);
        local_terminarPrograma = terminarPrograma;
        pthread_mutex_unlock(&terminarMutex);
    }

    // Informa sobre la terminación de la tarea
    printf("Tarea teclado terminada.\n");
    return NULL;
}

// Función para la tarea que lee del puerto serie
void* tareaLeerSerial(void* arg) {
    // Variables de comunicacion
    int wait_time = 0;
    int bytes_read = 0;
    int bytes_data = 0;
    char RX_buff [BUFF_SIZE];
    int delay_dsec = DEC_DELAY;



    // Variables para guardar datos
    int temp = -1;
    int hum = -1;
    int nivel = -1;
    int litros = -1;
    int litros_c = -1;
    int vaciado = 0;
    int local_terminarPrograma = 0;
    pthread_mutex_lock(&terminarMutex);
    local_terminarPrograma = terminarPrograma;
    pthread_mutex_unlock(&terminarMutex);
    while (!local_terminarPrograma) {
        // // Coloca aquí la lógica para leer del puerto serie
        pthread_mutex_lock(&serialMutex);
        if(wait_time == delay_dsec){
            // Comprobar si hay algo en el buffer de entrada
            ioctl(serial_port, FIONREAD, &bytes_read);
            // Si no hay nada, error
            if(bytes_read < 0) {
                printf("Error %i al leer: %s\n", errno, strerror(errno));
                // return 4;
                terminarPrograma = 1;
            }
            wait_time = 0;
            // Leer datos
            bytes_read = read(serial_port, RX_buff, BUFF_SIZE);
            // Si no se ha leido nada, error
            if(bytes_read < 0) {
                printf("Error %i al leer: %s\n", errno, strerror(errno));
                // return 4;
                terminarPrograma = 1;
            }
            // Hacer clear de la terminal, teniendo en cuenta que el cursor no esta en la posicion horizontal 0 y que se desplaza el cursor cuando se imprime
            printf("\033[2J\033[1;1H");
            printf("\033[s\033[10A");
            printf("Menu de comunicacion\n");
            printf("Bytes recibidos: %d\n", bytes_read);
            // Imprimir respuesta
            sscanf(RX_buff, "%d %d %d %d ", &temp, &hum, &nivel, &litros);
            printf("Temperatura: %d ºC \n", temp);
            printf("Humedad: %d %% \n", hum);
            printf("Nivel de agua: %d %% \n", nivel);
            printf("Litros caidos: %d ml/s \n", litros);
            
        }else{
            wait_time++;
        }
        pthread_mutex_unlock(&serialMutex);
        // Esperar 0.1 segundos
        usleep(100000);
        pthread_mutex_lock(&terminarMutex);
        local_terminarPrograma = terminarPrograma;
        pthread_mutex_unlock(&terminarMutex);
    }

    // Informa sobre la terminación de la tarea
    printf("Tarea leer serial terminada.\n");
    return NULL;
}

// Función para la tarea que escribe en el puerto serie
void* tareaEscribirSerial(void* arg) {
    int terminarEscritura = 0;
    char TX_buff [BUFF_SIZE];
    int delay_dsec = DEC_DELAY;
    int bytes_written = 0;
    int local_terminarPrograma = 0;
    memset(TX_buff, 0, BUFF_SIZE);
    pthread_mutex_lock(&terminarMutex);
    local_terminarPrograma = terminarPrograma;
    pthread_mutex_unlock(&terminarMutex);
    if (!local_terminarPrograma) {
        // Coloca aquí la lógica para escribir en el puerto serie
        pthread_mutex_lock(&serialMutex);
        printf("Escribiendo en el puerto serie...\n");
        bytes_written = sprintf(TX_buff, "%02X", delay_dsec);
        write(serial_port, TX_buff, bytes_written);
        pthread_mutex_unlock(&serialMutex);
        terminarEscritura = 1;
    }

    // Informa sobre la terminación de la tarea
    printf("Tarea escribir serial terminada.\n");
    return NULL;
}

int main() {

    // Crear hilos para cada tarea
    pthread_t hiloPrincipal, hiloTeclado, hiloLeerSerial, hiloEscribirSerial;

    pthread_create(&hiloPrincipal, NULL, tareaPrincipal, NULL);
    // Esperar 1 segundo para que la tarea principal se ejecute primero
    usleep(1000000);
    pthread_create(&hiloEscribirSerial, NULL, tareaEscribirSerial, NULL);
    pthread_create(&hiloTeclado, NULL, tareaTeclado, NULL);
    pthread_create(&hiloLeerSerial, NULL, tareaLeerSerial, NULL);
    

    // Esperar a que los hilos terminen (esto nunca debería suceder en un programa multihilo)
    pthread_join(hiloEscribirSerial, NULL);
    pthread_join(hiloTeclado, NULL);
    pthread_join(hiloPrincipal, NULL);
    pthread_join(hiloLeerSerial, NULL);
    
    // Cerrar puerto serie
    // if(serial_port != -1){
    //     if(tcsetattr(serial_port, TCSANOW, &tty_old) != 0) {
    //         printf("Error %i de tcsetattr: %s\n", errno, strerror(errno));
    //     }
    //     close(serial_port);
    // }

    return 0;
}
