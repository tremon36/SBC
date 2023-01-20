import java.io.IOException;
import java.net.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

public class Server {
    static final int PORT = 4999;

    public static void main(String[] args) {

        ServerSocket serverSocket = null;
        Socket socket = null;
        ThreadPoolExecutor handler = (ThreadPoolExecutor) Executors.newFixedThreadPool(10);

        try{
            serverSocket = new ServerSocket(PORT);
            System.out.println("SERVER INIT CORRECT");
        } catch(IOException ex){
            System.out.println("Puerto del servidor ocupado");
        }

        while(true){
            try{
                socket = serverSocket.accept();
            } catch (IOException ex){
                System.out.println("Error al establecer comunicacion");
            }
            handler.execute(new Communicator(socket));

        }
    }
}