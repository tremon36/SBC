package com.example.mapito_12;

import android.widget.Toast;

import java.io.PrintWriter;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Connector implements Runnable{
    public Socket socket;
    public boolean connected = false;
    public boolean tryingToConnect = false;

    public void connect(){
        tryingToConnect = true;
        ExecutorService executor = Executors.newSingleThreadExecutor();
        executor.execute(this);
    }

    @Override
    public void run() {
        try {
            this.socket = new Socket("35.193.184.100", 4999);
            Sender sender = new Sender(socket);
            sender.send("application");
            connected = true;
        }catch(Exception ex){
            connected = false;
            tryingToConnect = false;

        };

    }
}
