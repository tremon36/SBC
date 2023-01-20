import java.io.*;
import java.net.Socket;
import java.sql.*;
import java.util.ArrayList;
import java.util.Scanner;


public class Communicator implements Runnable{

    private final Socket cliente;
    private BufferedReader in;
    public OutputStream outputStream;
    public static Communicator esp32;
    public static Communicator application;


    public Communicator(Socket cliente){
        this.cliente = cliente;
        try {
            InputStream inputStream = cliente.getInputStream();
            outputStream = cliente.getOutputStream();
            in = new BufferedReader(new InputStreamReader(inputStream));

        } catch (IOException ex) {
            System.out.println("Imposible comunicarse con el cliente");
        }

    }


    public void run(){

        String line;

        try {
            line = in.readLine();
            if(line.contentEquals("esp32")){
                Communicator.esp32 = this;
                System.out.println("esp32 connected");
            } else {
                Communicator.application = this;
                System.out.println("application connected");
            }
        }catch (IOException ignored){};

        while (true) {
            try {
                 if (Communicator.application == this && esp32 != null) {
                    line = in.readLine();
                    System.out.println("recieved "+line.charAt(0));
                    byte[] to_send = new byte[1];
                    to_send[0] = (byte)line.charAt(0);
                    esp32.outputStream.write(to_send);

                }
            } catch (IOException ex) {
                System.out.println("EXCEPTION");
                if(Communicator.esp32 == this) {
                    Communicator.esp32 = null;
                    System.out.println("Exception on esp32 socket");
                    return;
                } else if (Communicator.application == this) {
                    Communicator.application = null;
                    System.out.println("Exception on application socket");
                    return;
                }

            }
        }
    }


}
