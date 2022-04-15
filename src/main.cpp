#include <micro_ros_arduino.h>


#include <brickfeeder_cmd/srv/brickfeederservice.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif 

#define PWM1_Ch 0
#define PWM1_Res 8
#define PWM1_Freq 40
static const int led_pin = LED_BUILTIN;


  int state = 0;
  int steptimer = 5;
  int read_ir_timer = 3;
  int motor_timer = 6;
  int obj_count = 0;
  int obj_dispensed = 0;
  unsigned long solenoid_off_timer = 1000;
  unsigned long solenoid_on_timer = 100;
  unsigned long begin_time;
  unsigned long end_time;
  unsigned long currentMillis;
  unsigned long previous_step_Millis = 0;
  unsigned long previous_ir_Millis = 0;
  unsigned long previous_motor_Millis = 0;
  boolean new_object = true;
  boolean new_job = false;
  boolean job_ran = false;

  int step_pin = 26;
  int dir_pin = 27;
  int enable_pin = 14;
  int solenoid_pin1 =18;
  int solenoid_pin2 = 19;
  int vibrator_pin1 = 17;
  int vibrator_pin2 = 16;
  int vibrator_enable = 4;
  int MS1_pin = 12;
  int MS2_pin = 32;
  int MS3_pin = 33;
  int ir_pin = 25;

  int out_ir = 23;
  int out_read = 22;
  int out_solenoid = 21; 

  //char agent_ip[15] = "192.168.0.101";
  //char SSID_name[15] = "FTTH_EX1052";
  //char SSID_psw[13] = "Edeksilrerv7";
  char agent_ip[15] = "192.168.0.100";
  char SSID_name[15] = "TP-Link_4DA2";
  char SSID_psw[10] = "65370148";

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;
rcl_wait_set_t wait_set;

brickfeeder_cmd__srv__Brickfeederservice_Request res;
brickfeeder_cmd__srv__Brickfeederservice_Response req;

//example_interfaces__srv__AddTwoInts_Response res;
//example_interfaces__srv__AddTwoInts_Request req;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){Serial.println("Failed at setup"); delay(1000);};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void read_serial(void *parameter)
{
  while(1)
  {
    if (new_job && obj_dispensed >= obj_count)//Stop dispensed object goal is reached.
    {
      job_ran = true;
      new_job = false;
      //ledcWrite(vibrator_enable, 0);
      Serial.println("Job done!");
      Serial.println("How many objects should be dispensed?");
    }
    digitalWrite(out_read, 1);
    vTaskDelay(3 / portTICK_PERIOD_MS);
    digitalWrite(out_read, 0);
  }

}

void read_ir(void *parameter)
{
  while(1)
  {
    if (new_job && state == 0) //Read IR every 3 ms
    {
      //Prevents multiple detections.
      if ( !digitalRead(ir_pin) && new_object ) //Detect new object.
      {
        Serial.println("Detected new object");
        new_object = false;
        digitalWrite(solenoid_pin1, HIGH);
        digitalWrite(solenoid_pin2, LOW); 
        state = 1;
        //detected_obstacle = true;
      }
    }
    digitalWrite(out_ir, 1);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    digitalWrite(out_ir, 0);
  }
}

void stepmotor_run(void *parameter)
{
  while(1)
  {
    if (new_job == true)
    {
    //Serial.println("stepping");
    digitalWrite(step_pin, 1);//step signaL ON
    digitalWrite(step_pin, 0);//OFF
    digitalWrite(vibrator_pin1, 1);
    digitalWrite(vibrator_pin2, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    //delay between steps-- 5ms is a common value that works well in full step mode
    }
    else
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
      digitalWrite(vibrator_pin1, 0);
      digitalWrite(vibrator_pin2, 0);
    }
  }
}

void solenoid(void *parameter)
{
  while(1)
  {
    switch (state)
    {
      case 1: //Begin the timer for how long the solenoid should be turned off.
        state = 2;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        break;
      case 2: //Turn solenoid on again after time is reached.
        if (digitalRead(solenoid_pin1))
        {

          digitalWrite(solenoid_pin1, LOW);
          digitalWrite(solenoid_pin2, LOW);
          state = 3;
        }
        break;
      case 3: //Begin a sleep timer to make sure the solenoid itself is not being detected as an object
        state = 4;
        vTaskDelay(100 / portTICK_PERIOD_MS);
        break;
      
      case 4: //After time is reached we can detect a new object.
          new_object = true;
          state = 0;
          obj_dispensed = obj_dispensed + 1;
          Serial.println("Dispensed an object");
        break;

      default:
        break;
    }
    digitalWrite(out_solenoid, 1);
    vTaskDelay(4 / portTICK_PERIOD_MS);
    digitalWrite(out_solenoid, 0);
  }
}

void service_callback(const void * req, void * res){
  brickfeeder_cmd__srv__Brickfeederservice_Request * req_in = (brickfeeder_cmd__srv__Brickfeederservice_Request *) req;
  brickfeeder_cmd__srv__Brickfeederservice_Response * res_in = (brickfeeder_cmd__srv__Brickfeederservice_Response *) res;
  //example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
  //example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;
  if (!new_job)
  {
      obj_dispensed = 0;
      //obj_count = req_in->a;
      obj_count = req_in->brickgoal;
      Serial.print("Now dispensing ");
      Serial.print(obj_count, DEC);
      Serial.println(" objects.");
      new_job = true;
      //ledcWrite(vibrator_enable, 60);
  }
  else if (job_ran)
  {
    //res_in->sum = obj_count;
    res_in->goaldone = true;
    job_ran = false;
  }
  //printf("Service request value: %d + %d.\n", (int) req_in->a, (int) req_in->b);

  //res_in->sum = req_in->a + req_in->b;
}

void motorPWM(void *parameter)
{
  while(1)
  {
    if (new_job == true)
    {
      vTaskDelay(5 / portTICK_PERIOD_MS);
      //digitalWrite(vibrator_enable, 1);
      //vTaskDelay(10 / portTICK_PERIOD_MS);
      //digitalWrite(vibrator_enable, 0);
      ledcWrite(PWM1_Ch, 20);
    }
    else
    {
      //digitalWrite(vibrator_enable, 0);
      ledcWrite(PWM1_Ch, 0);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }

  }
}


void setup() {
  Serial.begin(115600);
  Serial.println("Bootig up");
  set_microros_wifi_transports(SSID_name, SSID_psw, agent_ip, 8888);
  Serial.println("Wifi connected to");
  delay(1000); 

  allocator = rcl_get_default_allocator();
  Serial.println("created default allocator");
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("created init options"); 

  // create node
  //RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));
  RCCHECK(rclc_node_init_default(&node, "brickfeeder_client_rclc", "", &support));
  Serial.println("created node");
  
  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(brickfeeder_cmd, srv, Brickfeederservice), "/brickfeeder"));
  //RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/add_two_ints"));
  Serial.println("created service");
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  Serial.println("initiated executor");
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
  Serial.println("created executor");

  pinMode(step_pin, OUTPUT);//step pin
  //pinMode(5, OUTPUT);//direction pin
  pinMode(dir_pin, OUTPUT);//direction pin
  pinMode(enable_pin, OUTPUT);//enable pin
  pinMode(ir_pin, INPUT); //IR Read
  pinMode(solenoid_pin1, OUTPUT); //Solenoid pin
  pinMode(solenoid_pin2, OUTPUT); //Solenoid pin
  pinMode(out_ir, OUTPUT);
  pinMode(out_read, OUTPUT);
  pinMode(out_solenoid, OUTPUT);
  pinMode(vibrator_pin1, OUTPUT);
  pinMode(vibrator_pin2, OUTPUT);
  //pinMode(vibrator_enable, OUTPUT);


  //ledcAttachPin(vibrator_enable, 0);
  //ledcSetup(0, 1000, 8);
  //enables a4988 stepper drivers, pin 8, 0=enable, 1=disable
  digitalWrite(enable_pin, 0);
  //sets x axis direction
  digitalWrite(dir_pin, 0);
  digitalWrite(MS1_pin, 0); //MS1
  digitalWrite(MS2_pin, 0); //MS2
  digitalWrite(MS3_pin, 0); //MS3
  digitalWrite(solenoid_pin1, LOW); //
  digitalWrite(solenoid_pin2, LOW); //
  //digitalWrite(vibrator_enable, 1);
  ledcAttachPin(vibrator_enable, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);

  Serial.println("Starting loop");
  Serial.println("How many objects should be dispensed?");
  //analogWrite(6, 160);
  pinMode(led_pin, OUTPUT);

  xTaskCreatePinnedToCore(
    motorPWM,
    "motorPWM",
    2048,
    NULL,
    12,
    NULL,
    app_cpu);

  xTaskCreatePinnedToCore(
    stepmotor_run,
    "stepmotor_run",
    2048,
    NULL,
    12,
    NULL,
    app_cpu);

  xTaskCreatePinnedToCore(
    read_ir,
    "read_ir",
    2048,
    NULL,
    11,
    NULL,
    app_cpu);

    xTaskCreatePinnedToCore(
    read_serial,
    "read_serial",
    2048,
    NULL,
    11,
    NULL,
    app_cpu);

    xTaskCreatePinnedToCore(
    solenoid,
    "solenoid",
    2048,
    NULL,
    11,
    NULL,
    app_cpu);

}


void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}