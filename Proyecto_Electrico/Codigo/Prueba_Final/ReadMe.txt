Los pasos para correr el software son los siguientes:
    
1 En primer lugar se deben instalar o actualizar las librerías necesarias
para trabajar con el lenguaje de programación de C++.

  En la terminal se utiliza el siguiente comando:
  - sudo apt-get install build-essential
					
2 Seguido de esto se ingresa al directorio donde se encuentra el software creado.

  Dirección dentro del directorio del proyecto:
  - Proyecto_Electrico/Codigo/Prueba_Final

3 Se crea el ejecutable, esto se puede realizar con el makefile creado para
este propósito.

  En la terminal se utiliza el siguiente comando:
  - make clean all

4 Se dirige al directorio build creado por el makefile el cual contiene el
ejecutable.

5 Se estable una variable de entorno que posea la ruta de la biblioteca de NatNet.
   
   En la terminal se utiliza el siguiente comando:
   - export LD\_LIBRARY\_PATH=\$LD\_LIBRARY\_PATH:\{Se coloca la dirección de la biblioteca donde se encuentra el directorio lib.\}

6 Ahora se puede correr el ejecutable creado.
  En la terminal se utiliza el siguiente comando:
  - ./TrasmisionDatos