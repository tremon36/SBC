package com.example.mapito_12;

import android.os.Handler;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;

public class MainActivity extends AppCompatActivity {

    Button start,pause;
    Connector connector;
    TextView state;
    Handler handler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        start = findViewById(R.id.button_start);
        pause = findViewById(R.id.button_stop);
        state = findViewById(R.id.state_text);

        this.handler = new Handler();
        this.connector = new Connector();
        this.connector.connect();
        start.setOnClickListener((v) -> {
            if(connector.connected) {
                Sender sender = new Sender(connector.socket);
                sender.send("+");
                state.setText("RUNNING");
                state.setTextColor(getResources().getColor(R.color.blue_color_text));
            } else {
                handler.post(() -> connect_and_send(() -> {
                    Sender sender = new Sender(connector.socket);
                    sender.send("+");
                    state.setText("RUNNING");
                    state.setTextColor(getResources().getColor(R.color.blue_color_text));
                }));
            }
        });
        pause.setOnClickListener((v) -> {

            if(connector.connected) {
                Sender sender = new Sender(connector.socket);
                sender.send("-");
                state.setText("PAUSED");
                state.setTextColor(getResources().getColor(R.color.red_color_text));
            } else {
                handler.post(() -> connect_and_send(() -> {
                    Sender sender = new Sender(connector.socket);
                    sender.send("-");
                    state.setText("PAUSED");
                    state.setTextColor(getResources().getColor(R.color.red_color_text));
                }));
            }


        });
    }

    public void connect_and_send(Runnable send_command){
        handler.post(() -> {
            while(!connector.connected){
                if(!connector.tryingToConnect){
                    connector.connect();
                }
            }
            handler.post(send_command);

        });
    }


}