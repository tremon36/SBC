package com.example.mapito_12;

import java.io.PrintWriter;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Sender implements Runnable{
    private Socket s;
    private PrintWriter out;
    private String packet;
    public Sender(Socket s){
        this.s = s;
        try {
            out = new PrintWriter(s.getOutputStream(), true);
        } catch(Exception ignored){};
    }
    @Override
    public void run() {
        out.println(packet);
    }

    public void send (String toSend){
        if(s != null) {
            this.packet = toSend;
            ExecutorService executor = Executors.newSingleThreadExecutor();
            executor.execute(this);
        }
    }
}
