2)	#include <LiquidCrystal.h>
3)	#include <Wire.h>
4)	 
5)	#define in1 A3
6)	#define in2 10
7)	#define in3 12
8)	#define in4 13
9)	#define enA 3
10)	#define enB 11
11)	 
12)	const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
13)	LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
14)	 
15)	const int encoderD0 = A0;
16)	const float wheelDiameter = 6.7;
17)	 
18)	int pulse = 0;
19)	int previousPos = 0;
20)	float distance = 0.0;
21)	 
22)	const int MPU_addr = 0x68;
23)	int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
24)	int minVal = 265;
25)	int maxVal = 402;
26)	double x, y, z;
27)	 
28)	float distanceTraveled = 0.0;
29)	unsigned long startTime = 0;
30)	 
31)	bool postSpinActionInitiated = false;
32)	unsigned long postSpinStartTime = 0;
33)	bool postSpinDistanceMeasured = false;
34)	unsigned long postSpinStopTime = 0;
35)	 
36)	bool actionCompleted = false; // Add this at the global scope, near your other global variables
37)	bool rampSpeedUsed = false;  // Flag to track if ramp speed has been used
38)	 
39)	unsigned long rampStartTime = 0;  // Variable to store the start time
40)	bool rampActionCompleted = false; // Flag to ensure the stop and rotate action happens only once
41)	 
42)	void setup() {
43)	  Wire.begin();
44)	  Wire.beginTransmission(MPU_addr);
45)	  Wire.write(0x6B);
46)	  Wire.write(0);
47)	  Wire.endTransmission(true);
48)	  Serial.begin(9600);
49)	 
50)	  pinMode(encoderD0, INPUT);
51)	  lcd.begin(16, 2);
52)	  lcd.clear();
53)	  pulse = 0;
54)	 
55)	  pinMode(in1, OUTPUT);
56)	  pinMode(in2, OUTPUT);
57)	  pinMode(in3, OUTPUT);
58)	  pinMode(in4, OUTPUT);
59)	 
60)	  pinMode(enA, OUTPUT);
61)	  pinMode(enB, OUTPUT);
62)	 
63)	  pinMode(A2, INPUT);
64)	  pinMode(A1, INPUT);
65)	 
66)	  rampStartTime = millis(); // Initialize ramp start time
67)	  startTime = millis(); // Record the start time
68)	}
69)	double readMPU();
70)	void moveUpRamp();
71)	void Stop();
72)	void rotate360();
73)	void handlePostSpinActions();
74)	bool handleMovement(int rightSensor, int leftSensor);

75)	void loop() {
76)	    unsigned long currentMillis = millis();
77)	    int currentPos = digitalRead(encoderD0);
78)	 
79)	    if (currentPos != previousPos) {
80)	        pulse++;
81)	        previousPos = currentPos;
82)	    }
83)	 
84)	    double angleX = readMPU();
85)	    int LEFT_SENSOR = digitalRead(A2);
86)	    int RIGHT_SENSOR = digitalRead(A1);
87)	 
88)	    // Update distance traveled only when the robot is moving
89)	    if (!(LEFT_SENSOR == 1 && RIGHT_SENSOR == 1)) {
90)	        distanceTraveled = ((pulse * wheelDiameter * PI) / 40.0) - 0.53;
91)	    }
92)	 
93)	    // Use ramp speed for the first 2.5 seconds after starting
94)	    if (currentMillis - startTime < 2500) {
95)	        moveForwardFast();
96)	    } else {
97)	        // Handle normal movement and rotation logic
98)	        if (currentMillis - rampStartTime >= 1000 && !rampActionCompleted) {
99)	            Stop();
100)	            delay(4000);
101)	            rotate360();
102)	            rampActionCompleted = true; // Indicate the 360-degree spin is done
103)	        }
104)	 
105)	        // Handle actions after the 360-degree spin
106)	        if (rampActionCompleted && !actionCompleted) {
107)	            handlePostSpinActions();;
108)	        }
109)	 
110)	        // Update LCD only when the robot is moving
111)	        bool isMoving = handleMovement(RIGHT_SENSOR, LEFT_SENSOR);
112)	        if (isMoving) {
113)	            lcd.clear();
114)	            lcd.setCursor(0, 0);
115)	            lcd.print("Angle: ");
116)	            lcd.print(angleX);
117)	 
118)	            lcd.setCursor(0, 1);
119)	            lcd.print("D: ");
120)	            lcd.print(distanceTraveled);
121)	            lcd.print(" T: ");
122)	            lcd.print((currentMillis - startTime) / 1000);  // Time in seconds
123)	        }
124)	    }
125)	}

126)	double readMPU() {
127)	  Wire.beginTransmission(MPU_addr);
128)	  Wire.write(0x3B);
129)	  Wire.endTransmission(false);
130)	  Wire.requestFrom(MPU_addr, 14, true);
131)	 
132)	  AcX = Wire.read() << 8 | Wire.read();
133)	  AcY = Wire.read() << 8 | Wire.read();
134)	  AcZ = Wire.read() << 8 | Wire.read();
135)	 
136)	  int xAng = map(AcX, minVal, maxVal, -90, 90);
137)	  int yAng = map(AcY, minVal, maxVal, -90, 90);
138)	  int zAng = map(AcZ, minVal, maxVal, -90, 90);
139)	  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
140)	  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
141)	  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
142)	 
143)	  return x;
144)	}
145)	
146)	void forward() {
147)	  //analogWrite(enA, 73);
148)	  //analogWrite(enB, 73);
149)	  analogWrite(enA, 80);
150)	  analogWrite(enB, 80);
151)	  digitalWrite(in1, HIGH);
152)	  digitalWrite(in2, LOW);
153)	  digitalWrite(in3, LOW);
154)	  digitalWrite(in4, HIGH);
155)	}
156)	 
157)	void moveUpRamp() {
158)	  analogWrite(enA, 230);
159)	  analogWrite(enB, 230);
160)	  digitalWrite(in1, HIGH);
161)	  digitalWrite(in2, LOW);
162)	  digitalWrite(in3, LOW);
163)	  digitalWrite(in4, HIGH);
164)	}
165)	 
166)	void right() {
167)	  analogWrite(enA, 150);
168)	  analogWrite(enB, 80);
169)	  digitalWrite(in1, LOW);
170)	  digitalWrite(in2, HIGH);
171)	  digitalWrite(in3, LOW);
172)	  digitalWrite(in4, HIGH);
173)	}
174)	 
175)	void left() {
176)	  analogWrite(enA, 80); //2 right motors
177)	  analogWrite(enB, 150);  //left
178)	  digitalWrite(in1, HIGH);
179)	  digitalWrite(in2, LOW);
180)	  digitalWrite(in3, HIGH);
181)	  digitalWrite(in4, LOW);
182)	}
183)	 
184)	void Stop() {
185)	  digitalWrite(in1, LOW);
186)	  digitalWrite(in2, LOW);
187)	  digitalWrite(in3, LOW);
188)	  digitalWrite(in4, LOW);
189)	}
190)	 
191)	void rotate360() {
192)	  // Adjust these values to rotate your robot 360 degrees
193)	  //180
194)	  analogWrite(enA, 250); //2 right motors
195)	  analogWrite(enB, 250);  //left
196)	  digitalWrite(in1, HIGH);
197)	  digitalWrite(in2, LOW);
198)	  digitalWrite(in3, HIGH);
199)	  digitalWrite(in4, LOW);
200)	  delay(1500); // This delay determines how long the rotation lasts. Adjust as necessary.
201)	  // Stop the rotation
202)	  digitalWrite(in1, LOW);
203)	  digitalWrite(in2, LOW);
204)	  digitalWrite(in3, LOW);
205)	  digitalWrite(in4, LOW);
206)	}
207)	


208)	void handlePostSpinActions() {
209)	    static bool actionCompleted = false;  // Static flag to remember if action has been completed
210)	    unsigned long currentMillis = millis();
211)	 
212)	    // Check if actions have already been completed
213)	    if (actionCompleted) {
214)	        return; // Exit if action has already been completed
215)	    }
216)	 
217)	    // Start the post-spin action after a delay once the spin is completed
218)	    if (!postSpinActionInitiated && rampActionCompleted) {
219)	        postSpinStartTime = currentMillis + 1500; // 1.5 seconds after rotate360
220)	        postSpinActionInitiated = true;
221)	        pulse = 0; // Reset the pulse count for distance measurement
222)	    }
223)	 
224)	    // Measure distance after returning to ground level
225)	    if (postSpinActionInitiated && currentMillis >= postSpinStartTime && !postSpinDistanceMeasured) {
226)	        float currentDistance = ((pulse * wheelDiameter * PI) / 40.0) - 0.53;
227)	        if (currentDistance >= 120.0) {
228)	            Stop();
229)	            postSpinStopTime = currentMillis;
230)	            postSpinDistanceMeasured = true;
231)	        }
232)	    }
233)	 
234)	    // Stop for 3 seconds after reaching 120 cm
235)	    if (postSpinDistanceMeasured && currentMillis - postSpinStopTime >= 3000) {
236)	        postSpinActionInitiated = false;
237)	        postSpinDistanceMeasured = false;
238)	        actionCompleted = true; // Mark that the action is completed
239)	    }
240)	}
241)	
242)	bool handleMovement(int rightSensor, int leftSensor) {
243)	    if (rightSensor == 0 && leftSensor == 0) {
244)	        forward();
245)	        return true; // Robot is moving
246)	    } else if (rightSensor == 1 && leftSensor == 0) {
247)	        right();
248)	        return true; // Robot is moving
249)	    } else if (rightSensor == 0 && leftSensor == 1) {
250)	        left();
251)	        return true; // Robot is moving
252)	    } else if (rightSensor == 1 && leftSensor == 1) {
253)	        Stop();
254)	        return false; // Robot is stopped
255)	    }
256)	    return false;
257)	}
258)	
259)	void moveForwardFast() {
260)	    analogWrite(enA, 200);
261)	    analogWrite(enB, 200);
262)	    digitalWrite(in1, HIGH);
263)	    digitalWrite(in2, LOW);
264)	    digitalWrite(in3, LOW);
265)	    digitalWrite(in4, HIGH);
266)	}

