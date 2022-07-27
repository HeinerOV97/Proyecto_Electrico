/*
Trabajo basado en el codigo de ejemplo brindado por la empresa Optitrack para utilizar como base
al buscar la recepcion de datos enviados a traves de una red de area local en el sistema operativo
Ubuntu, se realizaron los cambios pertinentes en el codigo base brindado para que el software sea
capaz de realizar las tareas esperadas para el proyecto. El codigo de ejemplo utilizado se 
encuentre en la version 4.0 del SDK de NatNet, este se encuentra en la pagina:

https://optitrack.com/support/downloads/developer-tools.html

Este programa es capaz de conectar con el servidor de NatNet en Motive y recibir datos de los 
movimientos de un esqueleto o un cuerpo rigido y a partir de estos datos crear un archivo CSV
con ellos.

*/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

using namespace std;

#ifndef _WIN32
char getch();
#endif
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* ServidorElegido, void* ContextoUsuario );
void NATNET_CALLCONV ObtencionDeDatos(sFrameOfMocapData* datos, void* DatosUsuario);
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);
int ConnectClient();
void CreacionArchivoCuerpoRigido(int frame, float x, float y, float z, float qx, float qy, float qz, float qw);
void CreacionArchivoEsqueleto(int frame, int id, float x, float y, float z, float qx, float qy, float qz, float qw);
void resetClient();

static const ConnectionType TipoDeConexionPorDefecto = ConnectionType_Multicast;

NatNetClient* Cliente = NULL;

//  Define parametros para de la conexion asi como la version que utiliza el programa para el manejo de bits recibidos.
std::vector< sNatNetDiscoveredServer > ServidoresDescubiertos;
//  Se almacenan los datos de las direcciones del servidor y el cliente.
sNatNetClientConnectParams ParametrosConexion;
//  Direccion por defecto del envio de datos multicast.
char DireccionMulticastPorDefecto[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int EjemplosPorFrame = 0;
// Almacena datos del servidor version de Motive, version NatNet que soporta.
sServerDescription DescripcionDelServidor;


int main( int argc, char* argv[] )
{
    // Se muestra la version de NatNet que sera utilizada
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "Se utiliza NatNet ver. %d.%d.%d.%d\n", ver[0], ver[1], ver[2], ver[3] );

    // Se obtienen los mensajes de que se envian al servidor.
    NatNet_SetLogCallback( MessageHandler );

    // Se crea el cliente.
    Cliente = new NatNetClient();

    // Se reciben los datos del servidor,
    Cliente->SetFrameReceivedCallback( ObtencionDeDatos, Cliente );

    /*
    Si no se indican las direcciones IP del del cliente y el servidor en la
    linea de comandos se busca automaticamente los servidores conectados en la red.
    */
    if ( argc == 1 )
    {

#if 0
        const unsigned int TiempoDeEsperaParaServidores = 5 * 1000; // Se espera 5 segundos para respuesta del servidor.
        // Se obtiene informacion de los primeros 10 servidores encontrados.
        const int MaximoDeServidores = 10;
        sNatNetDiscoveredServer servers[MaximoDeServidores];
        int ServidoresEncontrados = MaximoDeServidores;
        NatNet_BroadcastServerDiscovery( servers, &ServidoresEncontrados );

#endif

        /* Si no se declaran las direcciones IP del servidor se buscan automaticamente
        los servidores conectados en la red de area local*/
        printf( "Presione el número correspondiente al servidores que se desea conectar.\n" );
        printf( "Presione Q para terminar el programa.\n\n" );

        // Se inicializa la estructura que contiene todas las funciones principales para conectar con el servidor.
        NatNetDiscoveryHandle ConfiguracionParaConectar;
        NatNet_CreateAsyncServerDiscovery( &ConfiguracionParaConectar, ServerDiscoveredCallback );

        // Se define con cual servidor encontrado se trabajara a partir de la opcion marcada por el usuario.
        while ( const int c = getch() )
        {
            if ( c >= '1' && c <= '9' )
            {
                const size_t IndiceServidor = c - '1';
                if ( IndiceServidor < ServidoresDescubiertos.size() )
                {
                    const sNatNetDiscoveredServer& ServidorElegidoParaConexion = ServidoresDescubiertos[IndiceServidor];

                    if ( ServidorElegidoParaConexion.serverDescription.bConnectionInfoValid )
                    {
                        // Se almacenan todos los paramteros del servidor con el que se realiza la conexion.
                        snprintf(

                            DireccionMulticastPorDefecto, sizeof DireccionMulticastPorDefecto,
                            "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                            ServidorElegidoParaConexion.serverDescription.ConnectionMulticastAddress[0],
                            ServidorElegidoParaConexion.serverDescription.ConnectionMulticastAddress[1],
                            ServidorElegidoParaConexion.serverDescription.ConnectionMulticastAddress[2],
                            ServidorElegidoParaConexion.serverDescription.ConnectionMulticastAddress[3]
                        );

                        ParametrosConexion.connectionType = ServidorElegidoParaConexion.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
                        ParametrosConexion.serverCommandPort = ServidorElegidoParaConexion.serverCommandPort;
                        ParametrosConexion.serverDataPort = ServidorElegidoParaConexion.serverDescription.ConnectionDataPort;
                        ParametrosConexion.serverAddress = ServidorElegidoParaConexion.serverAddress;
                        ParametrosConexion.localAddress = ServidorElegidoParaConexion.localAddress;
                        ParametrosConexion.multicastAddress = DireccionMulticastPorDefecto;
                    }
                    else
                    {
                        /* Si algun dato no es recibido el programa coloca valores predeterminados
                        para intentar conectarse.*/
                        ParametrosConexion.connectionType = TipoDeConexionPorDefecto;
                        ParametrosConexion.serverCommandPort = ServidorElegidoParaConexion.serverCommandPort;
                        ParametrosConexion.serverDataPort = 0;
                        ParametrosConexion.serverAddress = ServidorElegidoParaConexion.serverAddress;
                        ParametrosConexion.localAddress = ServidorElegidoParaConexion.localAddress;
                        ParametrosConexion.multicastAddress = NULL;
                    }

                    break;
                }
            }
            else if ( c == 'q' )
            {
                return 0;
            }
        }

        NatNet_FreeAsyncServerDiscovery( ConfiguracionParaConectar );
    }
    // Se almacenan las direcciones IP  ingresadas por el usuario si es necesario.
    else
    {
        ParametrosConexion.connectionType = TipoDeConexionPorDefecto;

        if ( argc >= 2 )
        {
            ParametrosConexion.serverAddress = argv[1];
        }

        if ( argc >= 3 )
        {
            ParametrosConexion.localAddress = argv[2];
        }
    }

    int iResult;

    // Se realiza la conexion con el servidor.
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error al inicializar al cliente.\n");
        return 1;
    }
    else
    {
        printf("Cliente inicializado con éxito.\n");
    }


	// Se realiza una prueba para verificar la conexion.
    void* response;
    int BitsPrueba;

    // Se solicita recibir la descripcion de datos.
    sDataDescriptions *pDataDefs = NULL;

    if ( pDataDefs )
    {
        NatNet_FreeDescriptions( pDataDefs );
        pDataDefs = NULL;
    }

	// Ingreso de opciones por teclado
	printf("\nEl cliente esta conectado al servidor.\n");
    printf("\nSe pueden ingresar las siguientes opciones por medio del teclado.\n");
    printf("\nq. Salir del programa\n");
    printf("\nr. Reiniciar el cliente\n");
    printf("\nm. Definir Multicast como tipo de conexion\n");
    printf("\nu. Definir Unicast como tipo de conexion\n");
    printf("\nc. Conectar al cliente\n");
    printf("\nd. Desconectar al cliente\n");
	bool bExit = false;
	while ( const int c = getch() )
	{
		switch(c)
		{
			case 'q': //Cierra el programa.
                bExit = true;
				break;
			case 'r': // Reinicia el cliente
				resetClient();
				break;
            case 'm': // Se cambia a tipo de conexion Multicast.
                ParametrosConexion.connectionType = ConnectionType_Multicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Conexion del cliente cambiada a tipo Multicast.\n\n");
                else
                    printf("Error cambiando la conexion del cliente a tipo Multicast.\n\n");
                break;
            case 'u': // Se cambia a tipo de conexion Unicast. (NO RECOMENDABLE)
                ParametrosConexion.connectionType = ConnectionType_Unicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Conexion del cliente cambiada a tipo Unicast.\n\n");
                else
                    printf("Error cambiando la conexion del cliente a tipo Unicast.\n\n");
                break;
            case 'c' : // Conectar al cliente
                iResult = ConnectClient();
                break;
            case 'd' : // Desconectar el cliente
                // Funciona correctamente con tipo de conexion Unicast.
                iResult = Cliente->SendMessageAndWait("Disconnect", &response, &BitsPrueba);
                if (iResult == ErrorCode_OK)
                    printf("Se ha desconectado del servidor.");
                break;
			default:
				break;
		}
		if(bExit)
			break;
	}

	if (Cliente)
	{
		Cliente->Disconnect();
		delete Cliente;
		Cliente = NULL;
	}

	return ErrorCode_OK;
}

// AQUI TERMINA EL MAIN

// Inician las funciones.
//Analizar
void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* ServidorElegido, void* ContextoUsuario )
{
    char serverHotkey = '.';
    if ( ServidoresDescubiertos.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + ServidoresDescubiertos.size());
    }

    printf( "[%c] %s %d.%d at %s ",
        serverHotkey,
        ServidorElegido->serverDescription.szHostApp,
        ServidorElegido->serverDescription.HostAppVersion[0],
        ServidorElegido->serverDescription.HostAppVersion[1],
        ServidorElegido->serverAddress );

    if ( ServidorElegido->serverDescription.bConnectionInfoValid )
    {
        printf( "(%s)\n", ServidorElegido->serverDescription.ConnectionMulticast ? "multicast" : "unicast" );
    }
    else
    {
        printf( "(No se pudo detectar una conexion automatica.)\n" );
    }

    ServidoresDescubiertos.push_back( *ServidorElegido );
}

// Funcion encargada de establecer la conexion como cliente.
int ConnectClient()
{
    // Se realiza la desconexion si existe algun servidor conectado.
    Cliente->Disconnect();

    // Se inicializa el cliente y se conecta al servidor.
    int ConfirmaConexion = Cliente->Connect( ParametrosConexion );

    // Si la conexion falla
    if (ConfirmaConexion != ErrorCode_OK)
    {
        printf("No es posible conectar al servidor. Codigo de error: %d.\n", ConfirmaConexion);
        return ErrorCode_Internal;
    }
    else
    {
        // Si la conexion es lograda.
        void* CuadrosPorSegundoPrueba;
        int BitsPrueba = 0;
        ErrorCode ConfirmaInformacionServidor = ErrorCode_OK;

        // Se muestra toda la informacion que se obtuvo del servidor.
        memset( &DescripcionDelServidor, 0, sizeof( DescripcionDelServidor ) );
        ConfirmaInformacionServidor = Cliente->GetServerDescription( &DescripcionDelServidor );
        if ( ConfirmaInformacionServidor != ErrorCode_OK || ! DescripcionDelServidor.HostPresent )
        {
            printf("No es posible conectar al servidor.\n");
            return 1;
        }
        printf("\nInformacion del servidor conectado:\n");
        printf("Aplicacion: %s (ver. %d.%d.%d.%d)\n", DescripcionDelServidor.szHostApp, DescripcionDelServidor.HostAppVersion[0],
            DescripcionDelServidor.HostAppVersion[1], DescripcionDelServidor.HostAppVersion[2], DescripcionDelServidor.HostAppVersion[3]);
        printf("Version de NatNet: %d.%d.%d.%d\n", DescripcionDelServidor.NatNetVersion[0], DescripcionDelServidor.NatNetVersion[1],
            DescripcionDelServidor.NatNetVersion[2], DescripcionDelServidor.NatNetVersion[3]);
        printf("IP Local:%s\n", ParametrosConexion.localAddress );
        printf("IP Servidor:%s\n", ParametrosConexion.serverAddress );


        // Se obtienen los cuadros por segundos a los que envia el servidor.
        ConfirmaInformacionServidor = Cliente->SendMessageAndWait("FrameRate", &CuadrosPorSegundoPrueba, &BitsPrueba);
        if (ConfirmaInformacionServidor == ErrorCode_OK)
        {
            float CuadrosPorSegundoResultado = *((float*)CuadrosPorSegundoPrueba);
            printf("Cuadros por segundo que envia el servidor: %3.2f\n", CuadrosPorSegundoResultado);
        }
        else
            printf("Error obteniendo los cuadros por segundo.\n");
    }
    return ErrorCode_OK;
}


// Funcion encargada de recibir los datos del servidor conectado.
void NATNET_CALLCONV ObtencionDeDatos(sFrameOfMocapData* datos, void* DatosUsuario)
{
    NatNetClient* pClient = (NatNetClient*) DatosUsuario;

    int i=0;

    printf("Cuadro Obtenido : %d\n", datos->iFrame);

    // Parametros de los datos recibidos por el Mocap
    bool Grabando = ((datos->params & 0x01)!=0);
    bool ModelosCambiados = ((datos->params & 0x02)!=0);
    if(Grabando)
        printf("RECORDING\n");
    if(ModelosCambiados)
        printf("Models Changed.\n");

	//  Cuerpos Rigidos encontrados.
	printf("Cuerpos rigidos [Encontrados=%d]\n", datos->nRigidBodies);
	for(i=0; i < datos->nRigidBodies; i++)
	{
        // Se rastrea el cuerpo rigido correctamente.
        bool RastreoValido = datos->RigidBodies[i].params & 0x01;

		printf("Cuerpo rigido [ID=%d  Error=%3.2f  Valido=%d]\n", datos->RigidBodies[i].ID, datos->RigidBodies[i].MeanError, RastreoValido);
		printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
			datos->RigidBodies[i].x,
			datos->RigidBodies[i].y,
			datos->RigidBodies[i].z,
			datos->RigidBodies[i].qx,
			datos->RigidBodies[i].qy,
			datos->RigidBodies[i].qz,
			datos->RigidBodies[i].qw);
        // Se crea el archivo CSV para el cuerpo rigido.
        CreacionArchivoCuerpoRigido(datos->iFrame, datos->RigidBodies[i].x,
        			datos->RigidBodies[i].y,
        			datos->RigidBodies[i].z,
        			datos->RigidBodies[i].qx,
        			datos->RigidBodies[i].qy,
        			datos->RigidBodies[i].qz,
        			datos->RigidBodies[i].qw);
	}

	// Esqueletos
	printf("Esqueletos [Encontrados=%d]\n", datos->nSkeletons);
	for(i=0; i < datos->nSkeletons; i++)
	{
		sSkeletonData DatosEsqueleto = datos->Skeletons[i];
		printf("Esqueleto [ID=%d  Segmentos encontrados=%d]\n", DatosEsqueleto.skeletonID, DatosEsqueleto.nRigidBodies);
		for(int j=0; j< DatosEsqueleto.nRigidBodies; j++)
		{
			sRigidBodyData DatosCuerpoRigido = DatosEsqueleto.RigidBodyData[j];
			printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				DatosCuerpoRigido.ID, DatosCuerpoRigido.x, DatosCuerpoRigido.y, DatosCuerpoRigido.z, DatosCuerpoRigido.qx, DatosCuerpoRigido.qy, DatosCuerpoRigido.qz, DatosCuerpoRigido.qw );
                CreacionArchivoEsqueleto(datos->iFrame, DatosCuerpoRigido.ID, DatosCuerpoRigido.x, DatosCuerpoRigido.y, DatosCuerpoRigido.z, DatosCuerpoRigido.qx, DatosCuerpoRigido.qy, DatosCuerpoRigido.qz, DatosCuerpoRigido.qw);
		}
	}
}


// Se encarga de recibir mensajes desde el servidor de NatNet.
void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    //  Si no es un mensaje de error
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}

void CreacionArchivoCuerpoRigido(int frame, float x, float y, float z, float qx, float qy, float qz, float qw)
{
    fstream Archivo;

    fstream ReadFile("CuerpoRigido.csv", ios::in);

    if(!ReadFile){
        cout<<"Creando Registro..."<<endl;
        fstream CreateFile("CuerpoRigido.csv", ios::out);
        CreateFile<<"frame, x, y, z, qx, qy, qz, qw"<<endl;
    }

    Archivo.open("CuerpoRigido.csv", ios::app);

    Archivo<<frame<<","<<x<<","<<y<<","<<z<<","<<qx<<","<<qy<<","<<qz<<","<<qw<<endl;

    Archivo.close();

}

void CreacionArchivoEsqueleto(int frame, int id, float x, float y, float z, float qx, float qy, float qz, float qw)
{
    fstream Archivo;

    fstream ReadFile("Esqueleto.csv", ios::in);

    if(!ReadFile){
        cout<<"Creando Registro..."<<endl;
        fstream CreateFile("Esqueleto.csv", ios::out);
        CreateFile<<"frame, Bone ID, x, y, z, qx, qy, qz, qw"<<endl;
    }

    Archivo.open("Esqueleto.csv", ios::app);

    Archivo<<frame<<","<<id<<","<<x<<","<<y<<","<<z<<","<<qx<<","<<qy<<","<<qz<<","<<qw<<endl;

    Archivo.close();

}


// Funcion encargada de reiniciar el cliente.
void resetClient()
{
	int ResultadoReinicio;

	printf("\n\nIniciando de nuevo la configuración del cliente\n\n.");

	ResultadoReinicio = Cliente->Disconnect();
	if(ResultadoReinicio != 0)
		printf("Error desconectando el cliente\n");

    ResultadoReinicio = Cliente->Connect( ParametrosConexion );
	if(ResultadoReinicio != 0)
		printf("Error reiniciando el cliente\n");
}


// Esta funcion se mantiene intacta en comparacion al codigo de ejemplo original
// esto porque permite la obtencion de entradas por teclado.
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    return buf;
}
