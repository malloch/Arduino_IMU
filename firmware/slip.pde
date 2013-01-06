/**************************************************************************
 *                                                                         *
 * SLIP-Encode Arduino output                                              *
 * 2011 Joseph Malloch / Input Devices and Music Interaction Laboratory    *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

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
