void setup() {
 Serial.begin(115200); // initialize serial communication at 9600 bits per second
}

void loop() {
 Serial.println("hello world"); // print "hello world" to the serial port
 delay(1000); // wait for a second
}