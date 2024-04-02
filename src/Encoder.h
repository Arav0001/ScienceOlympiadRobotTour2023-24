class encoder{
  private: 
    int enA, enA, pos1; 
    enA = 0;
    enB = 0;
    pos = 0;
  public: 
  encoder(int pinA, int pinB){
    enA = pinA;
    enB = pinB;
  }
  int readEncoder1(){
    if(digitalRead(enA)){
      -digitalRead(enB) ? pos1++ : pos1--;
    }
    return pos;
  }
}
