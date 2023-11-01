//超声波智能避障车程序（ARDUINO）
//    L = 左
//    R = 右
//    F = 前
//    B = 後

// #include <Servo.h> 
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_NeoPixel.h>
int pinLB=7;     // 定義7腳位 左後
int pinLF=8;     // 定義8腳位 左前

int pinRB=9;    // 定義9腳位 右後
int pinRF=10;    // 定義10腳位 右前

int pinEnA_Speed=3; //PWM
int pinEnB_Speed=11; //PWM

int inputPin = A0;  // 定義超音波信號接收腳位
int outputPin =A1;  // 定義超音波信號發射腳位

int Fspeedd = 0;      // 前速
int Rspeedd = 0;      // 右速
int Lspeedd = 0;      // 左速
int directionn = 0;   // 前=8 後=2 左=4 右=6 
// Servo myservo;        // 設 myservo
int pinLED_circle = 5;
int delay_time = 250; // 伺服馬達轉向後的穩定時間

int Fgo = 8;         // 前進
int Rgo = 6;         // 右轉
int Lgo = 4;         // 左轉
int Bgo = 2;         // 倒車

int buttonState = 0;
int lastButtonState = 0;

int powerstatus = 0;
int uptime_cnt = 0;
int uptime_cnt_max = 120;//seconds
int interval = 1000; //1000ms = 1s
int interruptPin = 2;

int ledPin = 13;
int ledPin2 = 12;

int buzzerPin = 6;

static int wakeupflag = 0;


#define NUMPIXELS 16 // 數量(可更改)
Adafruit_NeoPixel pixels( NUMPIXELS, pinLED_circle ); //設定腳位及數量(不可更改)
void setup()
 {
  Serial.begin(9600);     // 定義馬達輸出腳位 
  pinMode(pinLB,OUTPUT); // 腳位 8 (PWM)
  pinMode(pinLF,OUTPUT); // 腳位 9 (PWM)
  pinMode(pinRB,OUTPUT); // 腳位 10 (PWM) 
  pinMode(pinRF,OUTPUT); // 腳位 11 (PWM)
  
  pinMode(inputPin, INPUT);    // 定義超音波輸入腳位
  pinMode(outputPin, OUTPUT);  // 定義超音波輸出腳位   

  // myservo.attach(pinservo);    // 定義伺服馬達輸出第5腳位(PWM)

  pinMode(pinEnA_Speed,OUTPUT);//定义pinEnA_Speed为输出模式
  pinMode(pinEnB_Speed,OUTPUT);//定义pinEnB_Speed为输出模式
  analogWrite(pinEnA_Speed,200);//输入模拟值进行设定速度
  analogWrite(pinEnB_Speed,200);

  pinMode (interruptPin, INPUT_PULLUP);
  pinMode (ledPin, OUTPUT);
  pinMode (ledPin2, OUTPUT);

  pinMode (buzzerPin, OUTPUT);
  analogWrite(buzzerPin, 0);

  pixels.begin(); //啟用pixels服務

  

 }
void advance(int a)     // 前進
    {
     digitalWrite(pinRB,LOW);  // 使馬達（右後）動作
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // 使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     delay(a * 100);     
    }

void right(int b)        //右轉(單輪)
    {
     digitalWrite(pinRB,LOW);   //使馬達（右後）動作
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     delay(b * 100);
    }
void left(int c)         //左轉(單輪)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);   //使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     delay(c * 100);
    }
void turnR(int d)        //右轉(雙輪)
    {
     digitalWrite(pinRB,LOW);  //使馬達（右後）動作
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  //使馬達（左前）動作
     delay(d * 100);
    }
void turnL(int e)        //左轉(雙輪)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW);   //使馬達（右前）動作
     digitalWrite(pinLB,LOW);   //使馬達（左後）動作
     digitalWrite(pinLF,HIGH);
     delay(e * 100);
    }    
void stopp(int f)         //停止
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     delay(f * 100);
    }
void back(int g)          //後退
    {

     digitalWrite(pinRB,HIGH);  //使馬達（右後）動作
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //使馬達（左後）動作
     digitalWrite(pinLF,LOW);
     delay(g * 100);     
    }
    
void detection()        //測量3個角度(0.90.179)
    {      
      int delay_time = 250;   // 伺服馬達轉向後的穩定時間
      ask_pin_F();            // 讀取前方距離
      
     if(Fspeedd < 10)         // 假如前方距離小於10公分
      {
      stopp(1);               // 清除輸出資料 
      back(2);                // 後退 0.2秒
      }
           
      if(Fspeedd < 25)         // 假如前方距離小於25公分
      {
        // stopp(1);               // 清除輸出資料 
        // ask_pin_L();            // 讀取左方距離
        // delay(delay_time);      // 等待伺服馬達穩定
        // ask_pin_R();            // 讀取右方距離  
        // delay(delay_time);      // 等待伺服馬達穩定  
        
        // if(Lspeedd > Rspeedd)   //假如 左邊距離大於右邊距離
        // {
        //  directionn = Rgo;      //向右走
        // }
        
        // if(Lspeedd <= Rspeedd)   //假如 左邊距離小於或等於右邊距離
        // {
        //  directionn = Lgo;      //向左走
        // } 
        
        // if (Lspeedd < 10 && Rspeedd < 10)   //假如 左邊距離和右邊距離皆小於10公分
        // {
        //  directionn = Bgo;      //向後走        
        // } 

        directionn = Rgo; //force right only         
      }
      else                      //加如前方不小於(大於)25公分     
      {
        directionn = Fgo;        //向前走     
      }
     
    }    
void ask_pin_F()   // 量出前方距離 
    {
      // myservo.write(90);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Fdistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Fdistance= Fdistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("F distance:");      //輸出距離（單位：公分）
      Serial.println(Fdistance);         //顯示距離
      Fspeedd = Fdistance;              // 將距離 讀入Fspeedd(前速)
    }  
 void ask_pin_L()   // 量出左邊距離 
    {
      // myservo.write(5);
      delay(delay_time);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Ldistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Ldistance= Ldistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("L distance:");       //輸出距離（單位：公分）
      Serial.println(Ldistance);         //顯示距離
      Lspeedd = Ldistance;              // 將距離 讀入Lspeedd(左速)
    }  
void ask_pin_R()   // 量出右邊距離 
    {
      // myservo.write(177);
      delay(delay_time);
      digitalWrite(outputPin, LOW);   // 讓超聲波發射低電壓2μs
      delayMicroseconds(2);
      digitalWrite(outputPin, HIGH);  // 讓超聲波發射高電壓10μs，這裡至少是10μs
      delayMicroseconds(10);
      digitalWrite(outputPin, LOW);    // 維持超聲波發射低電壓
      float Rdistance = pulseIn(inputPin, HIGH);  // 讀差相差時間
      Rdistance= Rdistance/5.8/10;       // 將時間轉為距離距离（單位：公分）
      Serial.print("R distance:");       //輸出距離（單位：公分）
      Serial.println(Rdistance);         //顯示距離
      Rspeedd = Rdistance;              // 將距離 讀入Rspeedd(右速)
    }  

void sleepNow ()
{
  powerstatus = 0;
  //turn off ledpin
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin2, LOW);
  toneDown();

  ADCSRA = 0;                                                           //disable the ADC
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable ();                                                     // enables the sleep bit in the mcucr register
  noInterrupts();
  attachInterrupt (0, Wakeup_Routine, FALLING);          // wake up on RISING level on D2

  EIFR = bit (INTF0);  // clear flag for interrupt 0                                                                  

  // turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles
  MCUCR = bit (BODS); 
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle
  sleep_cpu ();   // one cycle
  
}  // end of sleepNow

//Wakeup routine, triggered pin 2
void Wakeup_Routine()
{
   //Internal Reset
  // asm volatile ("  jmp 0");

  
  powerstatus =1;
  wakeupflag = 1;

  // cancel sleep as a precaution
  sleep_disable();
  // precautionary while we do other stuff
  detachInterrupt (0);

}

void toneUp(){
  tone(buzzerPin, 261);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 396);
  delay(100);
  noTone(buzzerPin);


}

void toneDown(){
  tone(buzzerPin, 396);
  delay(100);
  noTone(buzzerPin);
  delay(100);
  tone(buzzerPin, 261);
  delay(100);
  noTone(buzzerPin);
  
}

void light_task(){
  static uint32_t tick,j=0;
  static char up_down_flag =0; //0 means up, 1 means down
  int R,G,B;
  if (millis() - tick >= 2)
  {
      
    for(int i=0; i<NUMPIXELS; i++){ //設定「每一顆」燈的顏色
      R = random(0,255);
      G = random(0,255);
      B = random(0,255);
			pixels.setPixelColor(i, pixels.Color( R, G, B )); //設定燈的顏色
		}
    if(up_down_flag == 0){
      j = j+2;
      if(j>=50){
        up_down_flag = 1;
      }
    }else{
      j = j -2;
      if(j<=0){
        j=0;
        up_down_flag = 0;
      }
    }
      
    pixels.setBrightness(j); //brightness
		pixels.show();

    // update next time
    tick = millis();
  
  }
}

void light_task2(){
  static uint32_t tick,i=0,j=0;
  int R,G,B;
  static char change_color_flag =1; //1 means change color
  static char up_down_flag =0; //0 means up, 1 means down
  if (millis() - tick >= 5)
  {
      if(change_color_flag){
        R = random(0,255);
        G = random(0,255);
        B = random(0,255);
        change_color_flag = 0;
      }
      pixels.setPixelColor(i, pixels.Color( R, G, B )); //設定燈的顏色

      i++;
      if (i >= NUMPIXELS){
        i=0;
        change_color_flag = 1;
      }
      if(up_down_flag == 0){
        j = j+2;
        if(j>=50){
          up_down_flag = 1;
        }
      }else{
        j = j -2;
        if(j<=0){
          j=0;
          up_down_flag = 0;
        }
      }
    pixels.setBrightness(j); //brightness
    pixels.show();
  }
}

void led_alive_task(){
  static uint32_t tick;
  static char led_status = 0;
  if (millis() - tick >= 500)
  {
    if(led_status){
      digitalWrite(ledPin, LOW);
      digitalWrite(ledPin2, HIGH);
      led_status = 0;
    }else{
      digitalWrite(ledPin, HIGH);
      digitalWrite(ledPin2, LOW);
      led_status = 1;
    }
    // update next time
    tick = millis();
  }


}
void loop()
 {
    //turn on ledpin
    //digitalWrite(ledPin, HIGH);
    // digitalWrite(ledPin2, HIGH);
    if(!powerstatus){
      pixels.setBrightness(0); //brightness
		  pixels.show();
      sleepNow();
      stopp(1);               // 清除輸出資料 

    }else{
      // myservo.write(90);  //讓伺服馬達回歸 預備位置 準備下一次的測量
      detection();        //測量角度 並且判斷要往哪一方向移動
      light_task2();
      led_alive_task();
      if(wakeupflag){
        toneUp();
        wakeupflag = 0;
        Serial.print("wakeupflag:" );   
        Serial.println(wakeupflag);
        
        power_all_enable();   // power everything back on
        pinMode(pinEnA_Speed,OUTPUT);//定义pinEnA_Speed为输出模式
        pinMode(pinEnB_Speed,OUTPUT);//定义pinEnB_Speed为输出模式
        analogWrite(pinEnA_Speed,125);//输入模拟值进行设定速度
        analogWrite(pinEnB_Speed,125);
      }
      if(directionn == 2)  //假如directionn(方向) = 2(倒車)             
      {
        back(8);                    //  倒退(車)
        turnL(2);                   //些微向左方移動(防止卡在死巷裡)
        Serial.println(" Reverse ");   //顯示方向(倒退)
      }
      if(directionn == 6)           //假如directionn(方向) = 6(右轉)    
      {
        back(1); 
        turnR(6);                   // 右轉
        Serial.println(" Right ");    //顯示方向(左轉)
      }
      if(directionn == 4)          //假如directionn(方向) = 4(左轉)    
      {  
        back(1);      
        turnL(6);                  // 左轉
        Serial.println(" Left ");     //顯示方向(右轉)   
      }  
      if(directionn == 8)          //假如directionn(方向) = 8(前進)      
      { 
        advance(1);                 // 正常前進  
        Serial.print(" Advance ");   //顯示方向(前進)
        Serial.println("   ");    
      }


      static uint32_t nextTime;

      // check if it's time
      if (millis() - nextTime >= interval)
      {
        // do your action here
        //increment uptime counter
        uptime_cnt++;
        Serial.print("uptime_cnt: ");
        Serial.println(uptime_cnt);

        // update next time
        nextTime += interval;
      }

      //check if uptime counter is greater than max
      if (uptime_cnt >= uptime_cnt_max)
      {
        //reset uptime counter
        uptime_cnt = 0;

        //do your action here
        Serial.println("reset uptime_cnt.. go to sleep");
        //enter sleep
        powerstatus =0;

      }

      buttonState = digitalRead(interruptPin);
      Serial.println(buttonState);
      // compare the buttonState to its previous state
      if (buttonState != lastButtonState) {
        // if the state has changed, increment the counter
        if (buttonState == LOW) {
        //   // if the current state is HIGH then the button went from off to on:
        //   buttonPushCounter++;
          Serial.println("button is pressed. sleeping...");
          //enter sleep
          delay(300);
          powerstatus = 0;
          
        }
          
          // Serial.print("number of button pushes: ");
          // Serial.println(buttonPushCounter);
          lastButtonState = buttonState;
      }

    }
    
 }


