#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31855.h>


// some configuration options



//#define USE_TWO_PROBES		// using two probes may be a cheap way to achieve better accuracy
				// We have enough data pins to use two

#define MAX_STEPS 64
#define COIL_SKIP_FACTOR 0	// 0 for no skip, 1 for 50% skip, 2 for 1/3 skip, N for 1/N+1 skip
				// Negative values for prevalent skip, e.g. -1 for 50% skip, -2 for 2/3 skip, -N for N/N+1 skip
				// use this if your coil is too powerful (e.g., 3000w)


//////// Start buttons library

// Modify these according to your analog keypad voltages
// Note this has been reordered to fit the latest analog keyboard
// I got from AliExpress. Looks like the new manufacturer changed the buttons 
// order and reference voltages.
enum ButtonKey
{
  KEY_LEFT = 0,
  KEY_UP,
  KEY_DOWN,
  KEY_RIGHT,
  KEY_SELECT,
  KEY_NONE,
  KEY_SIZE = KEY_NONE
};

// Modify these according to your analog keypad voltages
int KEYPAD_VOLTAGES[KEY_SIZE] = { 20, 170, 340, 530, 780 };

enum ButtonAction
{
  ACTION_NONE = 0,
  ACTION_DOWN,
  ACTION_UP,
  ACTION_CLICK = 4,
  ACTION_LONGCLICK = 8
};

struct ButtonHit
{
  ButtonKey key;
  ButtonAction action;

  static ButtonKey s_lastKey;
  static ButtonKey s_lastNonEmptyKey;
  static ButtonAction s_lastNonEmptyAction;
  static unsigned int s_keyDownSince;
};

ButtonKey ButtonHit::s_lastKey = KEY_NONE;
ButtonKey ButtonHit::s_lastNonEmptyKey = KEY_NONE;
ButtonAction ButtonHit::s_lastNonEmptyAction = ACTION_NONE;
unsigned int ButtonHit::s_keyDownSince = 0;


struct ButtonHit checkButton()
{
  int x;
  x = analogRead (A0);
  Serial.print( "read0=" );
  Serial.println(x);
  ButtonKey key = KEY_NONE;

  for (key=(ButtonKey)0; key < KEY_NONE; key = ButtonKey(key + 1))
  {
    if (x < KEYPAD_VOLTAGES[key])
      break;
  }

  struct ButtonHit ret;
  ret.key = key;
  ret.action = ACTION_NONE;
  if (ret.s_lastKey == key)
  {
    if (key != KEY_NONE)
      ++ret.s_keyDownSince;
  }
  else
  {
    ret.action = ACTION_UP;

    if (key != KEY_NONE)
    {
      ret.s_lastNonEmptyKey = key;
      ret.action = ACTION_DOWN;
    }
    else
    {
      // for button up we need to have a key
      ret.key = ret.s_lastKey;

      //see if we have click or longclick
      if (ret.s_keyDownSince > 3)
        ret.action = (ButtonAction)(ret.action | ACTION_LONGCLICK);
      else
        ret.action = (ButtonAction)(ret.action | ACTION_CLICK);
    }

    ret.s_lastKey = key;
    ret.s_keyDownSince = 0;
  }

  if (ret.action != ACTION_NONE)
    ret.s_lastNonEmptyAction = ret.action;  

  return ret;
}
/////      End buttons library

//////// Start generic utils
unsigned int DECIMAL_1(float x)
{
  return abs(int(round((x-(int)x)*10)));
}

int writeTemp(double temp, char *buf)
{
  int offset = sprintf(buf, "%d", (int)temp);
  // one decimal digit
  return offset + sprintf(buf+offset, ".%d", int((temp-int(temp))*10));
}

void fillLine(char *buf)
{
  int i = strlen(buf);
  for (; i<20; ++i)
    buf[i] = ' ';
}


//////// End generic utils


/////      Start common data structures
struct TemperatureStep
{
  short targetTemp; // in C
  unsigned char duration;	// minutes * 5, see DURATION_FACTOR
  bool  holdTemp;	// hold or increase 
}__attribute__ ((packed));

#define DURATION_FACTOR 5 // 5 mins for duration resolution

static struct TemperatureStep s_configuredSteps[MAX_STEPS];
static int s_configuredStepsCount = 0;


void resetSteps(struct TemperatureStep *steps)
{
  int i;
  for (i=0; i<MAX_STEPS; ++i)
  {
    steps[i].targetTemp = 0;
    steps[i].duration = 0;
    steps[i].holdTemp = true;
  }

  s_configuredStepsCount = 0;
}

// temperature sensor goes here
double tempC = 0;
double tempArr[2];
// sliding temp average
static const int AVG_TEMP_SAMPLES = 50;
double tempWindow[AVG_TEMP_SAMPLES] = {0.};
unsigned char nextSamplePos = 0;
double tempAverage = 0;

unsigned int s_ticks = 0;
unsigned long s_secondsAtStep = 0;	// when running steps
unsigned long s_msAtStep = 0;		// ditto
unsigned long s_msSinceStart = 0;	// updated in the loop
unsigned long s_msForCurrentStep = 0;	// mseconds when current step started

/////      End  common data structures


/////      Start SSR control logic
struct PID
{
  float i; // integral part
  bool heatOn;
  bool activateI;
  float tempIncrementRate; // computed increase rate for INCR mode
  float startTemp;         // starting temperature for INC mode
  unsigned long ticksRunning;
  static const float P_RATIO;
  static const float I_RATIO;
  static const float D_RATIO;	// inertia compensation: 
					// depends on how far the sensor is from the coil
  static const float D_RATIO_NEG;// smaller when temperature is decreasing.
};

const float PID::P_RATIO = 0.2f;
const float PID::I_RATIO = 0.0001f;
const float PID::D_RATIO = 100.f;  // inertia compensation: 
          // depends on how far the sensor is from the coil
const float PID::D_RATIO_NEG = 20.f;// smaller when temperature is decreasing.


struct PID s_PID;

const int SSRPIN = D2;
void initSSR()
{
  pinMode(SSRPIN, OUTPUT);
}

void toggleSSR(uint8_t value)
{
  {
    char buf[20] = "off";
    if (value == HIGH) sprintf(buf, "on");
    Serial.print("Turned SSR ");
    Serial.println(buf);
  }

  digitalWrite(SSRPIN, value);
  s_PID.heatOn = (value == HIGH);
}

void resetPID()
{
  s_PID.i = 0;
  s_PID.activateI = 0;
  s_PID.tempIncrementRate = 0;
  s_PID.startTemp = 0;
  s_PID.ticksRunning = 0;
}

/////      End   SSR control logic



/////      Start menu logic

enum MenuItems
{
  MENU_MONITOR,
  MENU_EDITSTEPS,
  MENU_RUNSTEPS,
  MENUITEMS_SIZE
};


struct EditSubmenuVars	// maps into char* SubMenu::vars
{
  char   nextAction; // add/remove/edit
  char   subAction;	// for add/edit, it is edit of step type, target temperature and time to reach
  bool   executingAction; // whether executing action above
  short  stepsCount;	// count of steps stored
  struct TemperatureStep steps[MAX_STEPS];	
};

struct SubMenu
{
  short stepId;
  char *vars = NULL;
};

enum EditSubmenuActions
{
  EDITACTION_ADD = 0,
  EDITACTION_REMOVE,
  EDITACTION_EDIT
};

enum EditSubmenuSubActions
{
  EDITSUBACTION_TYPE = 0,
  EDITSUBACTION_TEMP,
  EDITSUBACTION_DURATION
};


struct Menu
{
  int submenuId;
  struct EditSubmenuVars submenuEditSteps;
  struct SubMenu subMenus[MENUITEMS_SIZE];
};


int getStepDuration(const struct TemperatureStep *step)
{
  // duration is encoded in increments of 5 minutes
  return ((int)(step->duration)) * DURATION_FACTOR;
}

void resetStepsRun(struct Menu * menu)
{
  menu->subMenus[MENU_RUNSTEPS].stepId = 0;
  s_msForCurrentStep = s_msSinceStart;
  s_PID.heatOn = false;
  resetPID();
  // disable heating element for precaution.
  toggleSSR(LOW);
}

void initStep(struct Menu * menu)
{
  int stepId = menu->subMenus[MENU_RUNSTEPS].stepId;

  resetPID();

  if (stepId >= s_configuredStepsCount)
    return;

  s_msForCurrentStep = s_msSinceStart;

  if (!s_configuredSteps[stepId].holdTemp)
  {
    // set initial temp and temp. increase rate
    s_PID.startTemp = tempC;
    struct TemperatureStep * step = &s_configuredSteps[stepId];
    s_PID.tempIncrementRate = (step->targetTemp - s_PID.startTemp) / (getStepDuration(step) * 60.f);
  }
}

// returns target temp
//static char s_debugBuf[21];
float runStep(struct Menu * menu)
{
  short stepId = menu->subMenus[MENU_RUNSTEPS].stepId;
  float targetTemp = 0.f;
  //memset(s_debugBuf, 0, 21*sizeof(char));
  //fillLine(s_debugBuf);

  static float prevAverageTemp = -1.f;

  // actually perform running here
  s_msAtStep = (s_msSinceStart - s_msForCurrentStep);
  s_secondsAtStep = s_msAtStep / 1000;

  // do the PID stuff
  if (stepId < s_configuredStepsCount)
  {
    struct TemperatureStep * curStep = &s_configuredSteps[stepId];

    targetTemp = curStep->targetTemp;
    if (!curStep->holdTemp)
    {
      float stepDuration = s_msAtStep / 1000.f;
      targetTemp = s_PID.startTemp + stepDuration * s_PID.tempIncrementRate;
    }

    float dT = targetTemp - tempC;
    float p = dT * PID::P_RATIO;
    if (s_PID.activateI)
      s_PID.i += dT * PID::I_RATIO;
    else if(tempC >= targetTemp)
      s_PID.activateI = true;

    float d = 0.f;
    if (prevAverageTemp > -1.f)
    {
      float deltaT = (tempAverage - prevAverageTemp); // counter inertia 
      if (deltaT < 0)
        d = -deltaT * PID::D_RATIO_NEG; // sensor may report a sporadic decrease even if temp is actually increasing
					// hence NEG ratio should be less.
      else
        d = -deltaT * PID::D_RATIO;
    }
    prevAverageTemp = tempAverage;
    
    float action = p + s_PID.i + d;

    if (0)
    {
      char buf[80];
      sprintf(buf, "a=%d.%u, p=%d.%u, i=%d.%u, ",(int)action, DECIMAL_1(action), (int)p, DECIMAL_1(p), (int)s_PID.i, DECIMAL_1(s_PID.i));
      Serial.print(buf);
      sprintf(buf, "dT=%d.%u, tempC=%d, targetT=%d", (int)dT, DECIMAL_1(dT), (int)tempC, curStep->targetTemp );
      Serial.println(buf);
    }

    // prevent coil overloading and other effects (overshooting) if coil is too high watt rating.
    ++s_PID.ticksRunning;
    if (COIL_SKIP_FACTOR > 0 && (s_PID.ticksRunning % (COIL_SKIP_FACTOR+1))==0)
    {
      // skip heating this tick.
      //Serial.println("Skip heating coil due to COIL_SKIP_FACTOR");
      toggleSSR(LOW);
    }
    else if (COIL_SKIP_FACTOR < 0 && (s_PID.ticksRunning % (-(COIL_SKIP_FACTOR)+1))!=0)
    {
      // skip heating this tick (prevalent skip mode).
      toggleSSR(LOW);
    }
    else
    {
      //sprintf(s_debugBuf, "  a=%d.%u d=%d.%u", (int)action, DECIMAL_1(action), (int)d, DECIMAL_1(d));
      //Serial.println(s_debugBuf);
      //if (action >= 1 && !s_PID.heatOn)
      if (action > 0.1 && !s_PID.heatOn)
      {
        toggleSSR(HIGH);
      }
      else if (action < 1 && s_PID.heatOn)
      {
        toggleSSR(LOW);
      }
    }

    if (s_secondsAtStep >= ((unsigned long)getStepDuration(curStep))*60)
    {
      // move to next step.
      ++stepId;
      if (stepId < s_configuredStepsCount)
      {
        menu->subMenus[MENU_RUNSTEPS].stepId = stepId;
        initStep(menu);
      }
    } 
  }

  if (stepId >= s_configuredStepsCount)
  {
    // disable heating element.
    toggleSSR(LOW);
    menu->submenuId = MENU_MONITOR;
  }

  //s_debugBuf[0] = s_PID.heatOn ? '*' : ' ';
  return targetTemp;
}

void getCurrentMessage(struct Menu * menu, char **lines)
{
  static char buf[4][21];
  static const char * s_editActions[3] = { 
    "ADDSTEP", "RMSTEP", "EDITSTEP"           };
  memset(buf, 0, 21*4);
  switch(menu->submenuId)
  {
  case MENU_MONITOR:
    sprintf(buf[0], "[M] ");
    writeTemp(tempC, buf[0]+4);
    strcat(buf[0], "C  ");
    fillLine(buf[0]);
    lines[0] = buf[0];
    lines[1] = (char*)"";
    lines[2] = (char*)"";
    lines[3] = (char*)" <               >E ";
    break;	
  case MENU_EDITSTEPS:
    {
      EditSubmenuVars * vars = (EditSubmenuVars*)menu->subMenus[MENU_EDITSTEPS].vars;
      short stepId = menu->subMenus[MENU_EDITSTEPS].stepId;
      short stepToDisplay = min((short)(stepId+1), vars->stepsCount);
      sprintf(buf[0], "[E] %d/%d ", stepToDisplay, vars->stepsCount);
      strcat(buf[0], s_editActions[(int)vars->nextAction]);
      fillLine(buf[0]);
      lines[0] = buf[0];
      lines[1] = (char*)"";
      lines[2] = (char*)"";
      if (!vars->executingAction)
      {
        lines[3] = (char*)"M<               >R ";
      }
      else
      {
        switch(vars->subAction)
        {
          int ofs;
        case EDITSUBACTION_TYPE:
          sprintf(buf[3], "Type: " );
          strcat(buf[3], vars->steps[stepId].holdTemp ? "HOLD" : "INCR");
          break;
        case EDITSUBACTION_TEMP:
          ofs = sprintf(buf[3], "Tgt Temp: " );
          sprintf(buf[3]+ofs, "%dC", vars->steps[stepId].targetTemp);
          break;
        case EDITSUBACTION_DURATION:
          ofs = sprintf(buf[3], "Time: " );
          sprintf(buf[3]+ofs, "%dm", getStepDuration(&vars->steps[stepId]));
          break;
        }
        fillLine(buf[3]);
        lines[3] = buf[3];
      }
    }
    break;	
  case MENU_RUNSTEPS:
    {
      short stepId = menu->subMenus[MENU_RUNSTEPS].stepId;
      if (stepId >= s_configuredStepsCount)
      {
        menu->submenuId = MENU_MONITOR;
        return;
      }

      float targetTemp = runStep(menu);

      int i;
      short stepToDisplay = min(stepId+1, s_configuredStepsCount);
      int pos = sprintf(buf[0], "[R] %d/%d ", stepToDisplay, s_configuredStepsCount);
      if (stepId < s_configuredStepsCount)
      {
        struct TemperatureStep * curStep = &s_configuredSteps[stepId];
        pos += sprintf(buf[0]+pos, "%ld/%dm", s_secondsAtStep/60, getStepDuration(curStep));
        if (!curStep->holdTemp && pos < 14)
          pos += sprintf(buf[0]+pos, " I");
      }
      fillLine(buf[0]);
      lines[0] = buf[0];
      lines[1] = (char*)"";

      //sprintf(buf[2], (s_PID.heatOn ? "coilActive" : "          "));
      //sprintf(buf[2] + strlen(buf[2]), " i=%d.%d", (int)s_PID.i, DECIMAL_1(s_PID.i));
      //lines[2] = buf[2];

      //lines[2] = s_debugBuf;
      lines[2] = (char*)"";

      sprintf(buf[3], "E< ");
      writeTemp(tempC, buf[3]+3);
      if (stepId < s_configuredStepsCount)
      {
        struct TemperatureStep * curStep = &s_configuredSteps[stepId];
        int pos = strlen(buf[3]);
        if (curStep->holdTemp)
          sprintf(buf[3]+pos, "/%d", curStep->targetTemp);
        else
          sprintf(buf[3]+pos, "/%d", (int)round(targetTemp));
      }
      strcat(buf[3], "C");	

      for (i=strlen(buf[3]); i<17; ++i)
        buf[3][i] = ' ';
      strcat(buf[3], ">M ");
      lines[3] = buf[3]; 
    }
    break;	
  }
}

void activateStepsConfiguration(struct Menu * menu)
{
  // copy data to s_configuredSteps
  EditSubmenuVars * vars = (EditSubmenuVars*)menu->subMenus[MENU_EDITSTEPS].vars;
  s_configuredStepsCount = vars->stepsCount;
  memcpy(s_configuredSteps, vars->steps, sizeof(vars->steps));
}

void handleButtonsMonitor(struct Menu * menu, struct ButtonHit buttonHit)
{
  switch(buttonHit.key)
  {
  case KEY_RIGHT:
    menu->submenuId = MENU_EDITSTEPS;
    break;
  case KEY_UP:
    break;
  case KEY_DOWN:
    break;
  case KEY_LEFT:
    break;
  case KEY_SELECT:
    break;
  default:
    break;
  }
}

void handleButtonsEditSteps(struct Menu * menu, struct ButtonHit buttonHit)
{
  Serial.print("handleButtonsEditSteps: submenuId=");
  Serial.println(menu->submenuId);
  EditSubmenuVars * vars = (EditSubmenuVars*)menu->subMenus[menu->submenuId].vars;
  Serial.print("handleButtonsEditSteps: vars->nextAction=");
  Serial.println((int)vars->nextAction);
  int stepId = menu->subMenus[menu->submenuId].stepId;

  if (!vars->executingAction)
  {
    switch(buttonHit.key)
    {
    case KEY_RIGHT:
      activateStepsConfiguration(menu);
      resetStepsRun(menu);
      initStep(menu);
      menu->submenuId = MENU_RUNSTEPS;
      break;
    case KEY_UP:
      {
        int intNextAction = vars->nextAction;
        --intNextAction;
        if (intNextAction <0)
          intNextAction = 2;
        vars->nextAction = intNextAction;
      }
      break;
    case KEY_DOWN:
      ++vars->nextAction;
      if (vars->nextAction >2)
        vars->nextAction = 0;
      break;
    case KEY_LEFT:
      menu->submenuId = MENU_MONITOR;
      break;
    case KEY_SELECT:
      if (vars->nextAction == EDITACTION_ADD)
      {
        ++vars->stepsCount;
        if (vars->stepsCount > MAX_STEPS)
          vars->stepsCount = MAX_STEPS;
        else
        {
          menu->subMenus[menu->submenuId].stepId = vars->stepsCount-1;
          vars->subAction = EDITSUBACTION_TYPE;
          vars->executingAction = true;
        }
      }
      else if (vars->nextAction == EDITACTION_EDIT)
      {
        vars->executingAction = true;
      }
      else if (vars->nextAction == EDITACTION_REMOVE)
      {
        struct TemperatureStep * stepsArr = vars->steps;
        int i;
        vars->stepsCount = max(vars->stepsCount-1, 0);
        for (i=stepId; i < vars->stepsCount; ++i)
        {
          // move stuff
          memcpy(&stepsArr[i], &stepsArr[i+1], sizeof(TemperatureStep));
        }
        for (; i<MAX_STEPS; ++i)
        {
          memset(&stepsArr[i], 0, sizeof(TemperatureStep));
        }

        stepId = min(stepId, vars->stepsCount-1);
        menu->subMenus[menu->submenuId].stepId = stepId;
        activateStepsConfiguration(menu);
      }

      break;
    default:
      break;
    }
  }
  else if (vars->nextAction == EDITACTION_ADD ||
    vars->nextAction == EDITACTION_EDIT)
  {
    static int s_tempIncrement = 5, s_tempIncrementIdx = 0;
    static int s_tempIncrements[4] = { 
      5, 10, 50, 100             };

    struct TemperatureStep * curStep = &vars->steps[stepId];
    switch(buttonHit.key)
    {
    case KEY_UP:
      if (vars->subAction == EDITSUBACTION_TYPE)
        curStep->holdTemp = !curStep->holdTemp;
      else if (vars->subAction == EDITSUBACTION_TEMP)
      {
        if (buttonHit.action & ACTION_LONGCLICK && s_tempIncrementIdx < 3)
        {
          ++s_tempIncrementIdx;
          s_tempIncrement = s_tempIncrements[s_tempIncrementIdx];
        }
        curStep->targetTemp += s_tempIncrement;
      }
      else if (vars->subAction == EDITSUBACTION_DURATION)
        curStep->duration ++;

      break;
    case KEY_DOWN:
      if (vars->subAction == EDITSUBACTION_TYPE)
        curStep->holdTemp = !curStep->holdTemp;
      else if (vars->subAction == EDITSUBACTION_TEMP)
      {
        if (buttonHit.action & ACTION_LONGCLICK && s_tempIncrementIdx > 0)
        {
          --s_tempIncrementIdx;
          s_tempIncrement = s_tempIncrements[s_tempIncrementIdx];
        }
        curStep->targetTemp -= s_tempIncrement;
      }
      else if (vars->subAction == EDITSUBACTION_DURATION)
        curStep->duration --;

      curStep->duration = max(curStep->duration, (unsigned char)0);
      curStep->targetTemp = max(curStep->targetTemp, (short)0);

      break;
    case KEY_RIGHT:
      if (vars->nextAction == EDITACTION_EDIT)
      {
        // move to next step
        if (stepId < vars->stepsCount-1)
          ++menu->subMenus[menu->submenuId].stepId;
      }
      break;
    case KEY_LEFT:
      if (vars->nextAction == EDITACTION_EDIT)
      {
        // move to prev step
        if (stepId > 0)
          --menu->subMenus[menu->submenuId].stepId;
      }
      break;
    case KEY_SELECT:
      if (buttonHit.action & ACTION_LONGCLICK)
      {
        // save values and exit
        activateStepsConfiguration(menu);
        vars->executingAction = false;
      }
      else
      {
        ++vars->subAction;
        if (vars->subAction >2)
          vars->subAction = 0;
      }
      break;
    default:
      break;
    }

  }
}

void handleButtonsRunSteps(struct Menu * menu, struct ButtonHit buttonHit)
{
  switch(buttonHit.key)
  {
  case KEY_RIGHT:
    resetStepsRun(menu);
    menu->submenuId = MENU_MONITOR;
    break;
  case KEY_UP:
    break;
  case KEY_DOWN:
    break;
  case KEY_LEFT:
    resetStepsRun(menu);
    menu->submenuId = MENU_EDITSTEPS;
    break;
  case KEY_SELECT:
    break;
  default:
    break;
  }
}



void handleButtons(struct Menu * menu, struct ButtonHit buttonHit)
{
  if((buttonHit.action & ACTION_UP) == 0)
    return;

  switch(menu->submenuId)
  {
  case MENU_MONITOR:
    handleButtonsMonitor(menu, buttonHit);
    break;	
  case MENU_EDITSTEPS:
    handleButtonsEditSteps(menu, buttonHit);
    break;	
  case MENU_RUNSTEPS:
    handleButtonsRunSteps(menu, buttonHit);
    break;	
  }
}

void resetMenuMonitor(struct SubMenu *item)
{
  item->stepId = 0;
  // TODO: custom vars here
}

void resetMenuEditSteps(struct SubMenu *item)
{
  int i;
  EditSubmenuVars * vars = (EditSubmenuVars*)item->vars;
  item->stepId = 0;
  vars->stepsCount = 0;
  vars->nextAction = 0;
  vars->executingAction = false;

  for (i=0; i<MAX_STEPS; ++i)
  {
    vars->steps[i].targetTemp = 0;
    vars->steps[i].duration = 0;
    vars->steps[i].holdTemp = true;
  }
}

void resetMenuRunSteps(struct SubMenu *item)
{
  item->stepId = 0;
  // TODO: custom vars here
}



void resetMenuItem(struct Menu * menu, int item)
{
  switch(item)
  {
  case MENU_MONITOR:
    resetMenuMonitor(&menu->subMenus[item]);
    break;	
  case MENU_EDITSTEPS:
    resetMenuEditSteps(&menu->subMenus[item]);
    break;	
  case MENU_RUNSTEPS:
    resetMenuRunSteps(&menu->subMenus[item]);
    break;	
  }
}

void resetMenu(struct Menu * menu)
{
  int i;
  for(i=0; i<MENUITEMS_SIZE; ++i)
    resetMenuItem(menu, i);

  menu->submenuId = MENU_MONITOR;
}

/////      End menu logic


LiquidCrystal_I2C lcd( 0x27, 20, 4);
int ktc1_SO = D6;
int ktc1_CS = D7;
int ktc1_CLK = D5;
Adafruit_MAX31855 ktc1(ktc1_CLK, ktc1_CS, ktc1_SO);

#ifdef USE_TWO_PROBES
int ktc2_SO = 5;
int ktc2_CS = 6;
int ktc2_CLK = 7;
Adafruit_MAX31855 ktc2(ktc2_CLK, ktc2_CS, ktc2_SO);
#endif

struct Menu menu;


void setupMenu()
{
  menu.subMenus[MENU_EDITSTEPS].vars = (char*)&menu.submenuEditSteps;
  Serial.println("Resetting menu.");
  resetMenu(&menu);
}

void setup()
{ 
  Serial.begin(9600);
  initSSR();

  // give the MAX31855 a little time to settle
  delay(500);
  Serial.print("Initializing sensor(s)...");
  
  bool error = false;
  if (!ktc1.begin()) {
    Serial.println("ERROR communicating to temperature probe.");
    error = true;
  }
#ifdef USE_TWO_PROBES
  if (!ktc2.begin()) {
    Serial.println("ERROR communicating to temperature probe #2.");
    error = true;
  }
#endif

  if (error)
  {
    while (1) delay(10);
  }

  Serial.println("Initialization complete.");
  
  lcd.init();
  lcd.backlight();

  Serial.println("LCD setup complete.");
  setupMenu();
  Serial.println("Menu setup complete.");
  resetSteps(&s_configuredSteps[0]);
  Serial.println("Kiln setup done.");
}

double readTemp(double *tempOut)
{
  static double s_temp = 0;
  static double s_temp1 = 0;
  static double s_lastTemp = 0;
  
#ifdef USE_TWO_PROBES
  static double s_temp2 = 0;
#endif
  static int s_count = 0;

  if (s_count == 0)
  {
    s_temp1 = ktc1.readCelsius();
#ifdef USE_TWO_PROBES
    s_temp2 = ktc2.readCelsius();
    s_temp = (s_temp1+s_temp2)/2.;
#else
    s_temp = s_temp1;
#endif
  }

  ++s_count;
  if (s_count > 2)
    s_count = 0;

  if (isnan(s_temp))
  {
    return s_lastTemp;
  }

  if (tempOut)
  {
    tempOut[0] = s_temp1;
#ifdef USE_TWO_PROBES
    tempOut[1] = s_temp2;
#else
    tempOut[1] = 0;
#endif
  }

  s_lastTemp = s_temp;
  return s_temp;
}

void updateTempAverage()
{
  tempWindow[nextSamplePos] = tempC;
  ++nextSamplePos;
  if (nextSamplePos >= AVG_TEMP_SAMPLES)
    nextSamplePos = 0;

  double tempSum = 0;
  for (int i=0; i<AVG_TEMP_SAMPLES; ++i)
    tempSum += tempWindow[i];

  tempAverage = tempSum / AVG_TEMP_SAMPLES;
}


void loop()
{
  // your main loop code here...
  s_msSinceStart = millis();

  struct ButtonHit btn = checkButton();
  char * lines[4];

  tempC = readTemp(&tempArr[0]);
  updateTempAverage();


  Serial.print("Deg C = "); 
  Serial.print(tempC);
  Serial.print("; avg T=");
  Serial.println(tempAverage);
  Serial.print("t1=");
  Serial.print(tempArr[0]);
  Serial.print("; t2=");
  Serial.println(tempArr[1]);

  

  handleButtons(&menu, btn);
  getCurrentMessage(&menu, &lines[0]);
  for (int i=0; i<4; ++i)
  {
    lcd.setCursor(0,i);
    lcd.print(lines[i]);
  }

  lcd.setCursor(14,1);
  //switch(btn.s_lastNonEmptyKey)
  switch(btn.key)
  {
  case KEY_RIGHT:
    lcd.print ("Right ");
    break;
  case KEY_UP:
    lcd.print ("Up    ");
    break;
  case KEY_DOWN:
    lcd.print ("Down  ");
    break;
  case KEY_LEFT:
    lcd.print ("Left  ");
    break;
  case KEY_SELECT:
    lcd.print ("Select");
    break;
  case KEY_NONE:
    //lcd.print ("      ");
    break;

  }

  if (btn.key != KEY_NONE)
  {
    lcd.setCursor(13,3);
    //const char *clickStr = btn.s_lastNonEmptyAction == ACTION_NONE ? " " : (btn.s_lastNonEmptyAction == ACTION_DOWN ? "+" : "-");
    const char *clickStr = btn.action == ACTION_NONE ? " " : (btn.action == ACTION_DOWN ? "+" : "-");
    lcd.print(clickStr);
  }

  delay(100);
  ++s_ticks;
}
