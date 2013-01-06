void slipOutByte(byte output)
{
  if((output==escapeChar)||(output==delimiterChar)) Serial.write(escapeChar);
  Serial.write(output);
}

void slipOutInt(int output)
{
  slipOutByte((byte)(output >> 8));
  slipOutByte((byte)(output & 0xFF));
}

void slipOutDouble(double output)
{
  byte *b = (byte *)&output;
  for(int i=0; i<4; i++) {
    slipOutByte(b[i]);
  }
}