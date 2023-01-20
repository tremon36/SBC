using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class movimiento : MonoBehaviour
{
    // Start is called before the first frame update
    GameObject robot;
    float timer;
    float rotation_speed = 30.0f;
    float speed = 0.5f;
    public float rotation_accumulated;
    Vector3 direction;
    int currentRadarAngle;
    Vector3 recommended_path;
    distance_angle[] distancias;
    bool rotating = false;
    

    bool radar_arriba;
    
    class distance_angle {
        public float distancia;
        public float angulo;

        public distance_angle(float distancia, float angulo){
            this.angulo = angulo;
            this.distancia = distancia;
        }
    }
    void Start()
    {   robot = GameObject.Find("robot");
        timer = 3.0f;
        radar_arriba = true;
        currentRadarAngle = -60;
        distancias = new distance_angle[120];
        for(int i = 0; i < distancias.Length; i++){
            distancias[i] = new distance_angle(10000,0);
        }
        
        
    }

    // Update is called once per frame
    void Update()
    {   
        robot.transform.Translate(0,0,Input.GetAxis("Vertical") * speed * Time.deltaTime);
        robot.transform.Rotate(0,Input.GetAxis("Horizontal") * rotation_speed * Time.deltaTime,0);
        /*for(int i = 0; i < distancias.Length; i++){
                   distancias[i].angulo -= Input.GetAxis("Horizontal") * rotation_speed * Time.deltaTime;  
                }*/


       // average_vector_from_hits(distancias);
        
        
        
        recommended_path = average_vector_from_hits(distancias);
        Vector3 orientation = getOrientationVector(robot.transform.rotation,0);

        float angulo = Vector3.Angle(recommended_path,orientation);
        

        
        if(angulo > 0.5f){ // producto vectorial
            rotating = true;
            float producto_vectorial = orientation.x*recommended_path.z - orientation.z*recommended_path.x;
            Debug.Log("producto: " + producto_vectorial);
            if(producto_vectorial > 0){ //producto vectorial
                robot.transform.Rotate(new Vector3(0,-rotation_speed * Time.deltaTime,0));
                for(int i = 0; i < distancias.Length; i++){
                   distancias[i].angulo += rotation_speed * Time.deltaTime;  
                }
            }
            else{
                robot.transform.Rotate(new Vector3(0,rotation_speed * Time.deltaTime,0));
                for(int i = 0; i < distancias.Length; i++){
                   distancias[i].angulo -= rotation_speed * Time.deltaTime;  
                }
            } 
            
        } else{
            rotating = false;
            robot.transform.Translate(new Vector3(0,0,speed * Time.deltaTime));
         
        }
         Debug.DrawLine(robot.transform.position,robot.transform.position + recommended_path * 10,Color.yellow,Time.deltaTime);
    }

    private Vector3 getOrientationVector(Quaternion rotation, float offset_angle){
            float x = Mathf.Sin(2 * Mathf.PI * (robot.transform.rotation.eulerAngles.y + offset_angle) / 360.0f);
            float z = Mathf.Cos(2 * Mathf.PI * (robot.transform.rotation.eulerAngles.y + offset_angle) / 360.0f);
            return new Vector3(x,0,z);
    }

    void FixedUpdate() {

        for(int i = 0; i < 5; i++){
        RaycastHit hit;
        
        Physics.Raycast(robot.transform.position,getOrientationVector(robot.transform.rotation, currentRadarAngle ),out hit);
        
        //Debug.Log(hit.distance);
        
        if(radar_arriba) currentRadarAngle = currentRadarAngle + 1;
        else currentRadarAngle = currentRadarAngle - 1;
        if(currentRadarAngle >= 59){
                radar_arriba = false;
               // recommended_path = average_vector_from_hits(distancias);
        } 
        if(currentRadarAngle <= -60){
             radar_arriba = true;
            // recommended_path = average_vector_from_hits(distancias);
        }
       // Debug.Log(currentRadarAngle);
       
        distancias[currentRadarAngle + 60] = new distance_angle(hit.distance,currentRadarAngle); // habra que restar 60 mas adelante
        
        
        
         Debug.DrawLine(robot.transform.position,robot.transform.position + getOrientationVector(robot.transform.rotation, currentRadarAngle) * 5 ,Color.green,Time.deltaTime);
        //Debug.DrawLine(robot.transform.position,robot.transform.position + getOrientationVector(robot.transform.rotation, 180) * 5 ,Color.red,Time.deltaTime);
        //Debug.Log(getOrientationVector(robot.transform.rotation, -90));
        }
        Debug.DrawLine(robot.transform.position, robot.transform.position + getOrientationVector(robot.transform.rotation,0) * 5,Color.blue,Time.deltaTime);
        
        
    }

    private Vector3 average_vector_from_hits( distance_angle [] hit_array){
        int min_pos = 0;
        int inc = 0;
        distance_angle min_value = new distance_angle(10000,10000);
        for(int i = 0; i < hit_array.Length; i++){
            if(hit_array[i].distancia < min_value.distancia){
                min_value = hit_array[i];
                min_pos = i;
            }
        }
        //Debug.Log("pos = "+min_pos + " angulo: "+min_value.angulo);

        if(min_pos < hit_array.Length / 2){
            inc = 1;
        } else inc = -1;
        Vector3 siguiente_elemento = new Vector3(hit_array[min_pos+inc].distancia * Mathf.Sin((hit_array[min_pos+inc].angulo + robot.transform.rotation.eulerAngles.y) * Mathf.PI / 180.0f),0,hit_array[min_pos+inc].distancia * Mathf.Cos((hit_array[min_pos+inc].angulo + robot.transform.rotation.eulerAngles.y)* Mathf.PI / 180.0f));
        
        //Debug.Log(min_value.distancia + "," + min_value.angulo + " deg");
        Vector3 aElementoMenor = new Vector3(min_value.distancia * Mathf.Sin((min_value.angulo + robot.transform.rotation.eulerAngles.y) * Mathf.PI / 180.0f),0,min_value.distancia * Mathf.Cos((min_value.angulo + robot.transform.rotation.eulerAngles.y)* Mathf.PI / 180.0f));
        Debug.DrawLine(robot.transform.position,robot.transform.position + aElementoMenor ,Color.black,Time.deltaTime);
        Debug.DrawLine(robot.transform.position,robot.transform.position + siguiente_elemento ,Color.magenta,Time.deltaTime);

        if(min_value.distancia > 2) return aElementoMenor; 
        return (siguiente_elemento - aElementoMenor).normalized;
        



        
        
    }

  

   
}
