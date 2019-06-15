void readReceiver(){
  noInterrupts();
  sticksNew = true;
  switchesNew = true;
  
  //get receiver inputs as a percentage of max (assuming 1000-2000 us pulse range)
  recThrot = (return_THROT()-1000)/10;
  recRudd = (return_RUDD()-1000)/10;
  recElev = (return_AILER()-1000)/10;
  recAiler = (return_ELEV()-1000)/10;
  recGear = (return_GEAR()-1000)/10;
  recAiler = (return_FLAP()-1000)/10;
  interrupts();
}

//limit throttle pulse widths to 1000-2000ms

byte signalLost()
{
  if(RC_THROT < 995) return 1;
  else return 0;
  //Bind ESC with throttle travel at absolute minimum, below 1000ms pulses
  //then return travel to normal.  On losing signal it will give an abnormal result.
}
   
int volatile return_THROT()
{
	  return min(2000, RC_THROT); //max(1000, min(2000, RC_THROT));
}

int volatile return_AILER()
{
	  return max(1000, min(2000, RC_AILER));
}

int volatile return_ELEV()
{
	  return max(1000, min(2000, RC_ELEV));
}

int volatile return_RUDD()
{
	  return max(1000, min(2000, RC_RUDD));
}

int volatile return_GEAR()
{
	  return max(1000, min(2000, RC_MODE));
}

int volatile return_FLAP()
{
	  return max(1000, min(2000, RC_AUX1));
}
