int loops;
char test[20];

void setup() {
  Serial.begin(9600);
}

void loop() {

  if (loops>9){
    loops= 0;
  }
  testFunction(loops, test);
  Serial.println(0);
  Serial.println('0');
  for(int i; i<sizeof(test)){
    Serial.println(test[i]);
  }

  String s= "";
  s=strcat(s,test[3]); 
  s= strcat(s,"5");
  Serial.println(atoi(s));
  Serial.println(atol(test));
  Serial.println();
  loops++;
  delay(200);

}

void testFunction(char test[]){
  int test_size= sizeof(test);
  int i;
  while ( i< loops && i< test_size){
    test[i]= i;
    i++;
  }
  
}
