template<class T>
void MovingAverage<T>::giveAVGValue(T getdata)
{
  sum -= value[index];
  value[index] = getdata;
  sum += value[index];
//******print value for debug***********************
//  for (uint16_t i = 0; i < numreading; i++)
//  {
//    Serial.print(value[i]);   if(i == index) Serial.print("*"); Serial.print("\t");
//    if ((i % 20) == 19)       Serial.print("\n");
//  }
//  Serial.print("\n");
//**************************************************
  index++;
  if (index >= numreading)  {index = 0; first_time = false;}
}

template<class T>
void MovingAverage<T>::getAVGValue(T *avg_value)
{
  if (first_time == true)
    *avg_value = (T)((float)sum / index);
  else
    *avg_value = (T)((float)sum / numreading);
//******print value for debug***********************
  for (uint16_t i = 0; i < numreading; i++)
  {
    Serial.print(value[i]);   if(i == (index - 1)) Serial.print("*");   Serial.print("\t");
    if ((i % 20) == 19)       Serial.print("\n");
  }
  Serial.print("\n");
//**************************************************
}
