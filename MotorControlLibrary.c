#define 	ONE_OVER_SQRT3     		(float)0.5773502691    // Defines value for 1/sqrt(3)
#define 	SQRT3    		        (float)1.7320508076    // Defines value for sqrt(3)
#define     TWO_OVER_SQRT3          (float)1.1547005383    // Defines value for 2/sqrt(3)
#define     SQRT3_OVER_TWO          (float)0.86602540378   // Defines values for sqrt(3)/2
#define 	ANGLE_2PI              	(float)6.2831853072    // Defines value for 2*PI
#define     LUT_SIZE                256U                   // Defines size of sine and cosine tables
#define 	ANGLE_RES      		    (float)(ANGLE_2PI/(float)TABLE_SIZE) //Defines the angle resolution in the sine/cosine look up table
#define     ONE_OVER_ANGLE_RES     (float)(1.0f/ANGLE_RES)

// Structure containing component values for 3-Phase Stator Reference Frame
typedef struct 
{
    float   a;
    float   b; 
    float   c;
} struct_ABC;


// Structure containing component values for 2-Phase Stator Reference Frame
typedef struct 
{
    float   alpha;
    float   beta; 
} struct_AlphaBeta;


// Structure containing component values for 2-Phase Rotating Reference Frame
typedef struct 
{
    float   d;          // in direction parallel to flux (direct)
    float   q;          // in direction parallel to torque (quadrature)
} struct_DQ;


// Enum for Space Vector PWM sectors
enum sector {
    SECTOR1, // ABC = 100
    SECTOR2, // ABC = 110
    SECTOR3, // ABC = 010
    SECTOR4, // ABC = 011
    SECTOR5, // ABC = 001
    SECTOR6  // ABC = 101
            // sector 7 = 000
            // sector 8 = 111
}; 

// Structure containing duty values for phases/switches
typedef struct
{
    float a; // change struct depending on PWM mode of mcu
    float b;
    float c;
} struct_Duty;

// Structure containing variables used by PI Controller
typedef struct 
{
    float   sum;    // Sum of error values
    float   kp;     // Proportional Coefficient
    float   ki;     // Integral Coefficient
    float   kw;     // Anti-windup Coefficient for integral action
    float   outMax; // Max output limit (inclusive)
    float   outMin; // Min output limit (inclusive)
    float   ref;    // Reference input
    float   fdb;    // Feedback input
    float   out;    // Controller output (integral output + proportional output)
    float   error;  // Calculated error (reference - feedback)
} struct_PI;


/* Clears PI controller integral sum and output 
    Parameters:
        *PI          pointer to structure for PI controller values
*/
static inline void ClearPI(struct_PI *PI)
{
    PI->sum = 0.0f;
    PI->qOut = 0.0f;
}


/* Updates PI controller integral sum and output 
    Parameters:
        *PI          pointer to structure for PI controller values
*/
static inline void UpdatePI(struct_PI *PI)
{
    float errorTemp;
    float outTemp;
    float windup_delta;
    
    errorTemp  = PI->ref - PI->fdb;
    PI->error =  errorTemp; 
    outTemp  = PI->sum + PI->kp * errorTemp;
   
    // check output limits and assign output
    if(outTemp > PI->outMax)
    {
        PI->out = PI->outMax;
    }    
    else if(outTemp < PI->outMin)
    {
        PI->out = PI->outMin;
    }
    else        
    {
        PI->qOut = outTemp;  
    }
    
    windup_delta = outTemp - PI->out; // setpoint change 
    
    // calculate integral sum with anti-windup https://en.wikipedia.org/wiki/Integral_windup
    // when large setpoint changes happen, integral error sum won't include the error during ramp/windup
    PI->sum = PI->sum + (PI->ki * Err) - (PI->kw * windup_delta); 

}


/* Calculates Space Vector Sector (1-6) based on alpha beta components 
    Parameters:
        *alphabetaValues           pointer to structure for alpha,beta axis components
    Returns:
        sector                     Space Vector PWM sector enum (SECTOR1....SECTOR6)
*/
static inline sector SpaceVectorSector(struct_AlphaBeta *voltage)
{
    // each Space Vector sector is 60deg. Dividing lines between sectors is where y = tan(60) * x = sqrt(3) * x
    temp = voltage->alpha * SQRT3;

    // check each of four quadrant in alpha beta reference frame (alpha = x, beta = y)
    // determine Sector using 60deg property

    if ((voltage->alpha >= 0) && (voltage->beta >= 0)) // Quadrant 1
    {
        if (voltage->beta <= temp)
        {
            sector = SECTOR1;
        }
        else
        {
            sector = SECTOR2;
        }
    }
    else if ((voltage->alpha < 0) && (voltage->beta >= 0)) // Quadrant 2
    { 
        if (voltage->beta >= -tempIQ)
        {
            sector = SECTOR2;
        }
        else
        {
            sector = SECTOR3;
        }
    }
    else if ((voltage->alpha < 0) && (voltage->beta < 0)) // Quadrant 3
    {
        if (voltage->beta >= tempIQ)
        {
            sector = SECTOR4;
        }
        else
        {
            sector = SECTOR5;
        }
    }
    else                                                  // Quadrant 1
    {
        if (voltage->beta  <= -tempIQ)
        {
            sector = SECTOR5;
        }
        else
        {
            sector = SECTOR6;
        }
    }

    return sector;

}


/* Calculates Space Vector PWM Duties and Timing for six switches based on desired voltage vector  
    Parameters:
        *alphabetaValues           pointer to structure for alpha,beta axis components
        sector                     sector of desired voltage
        duty                       duty values for inverter switches
*/
static inline void SpaceVectorPWMDuty(struct_AlphaBeta alphabetaVoltage, sector sector, struct_Duty* duty)
{
    struct_ABC abcVoltages;
    
    InverseClarkeTransform(&alphabetaVoltage, &abcVoltages)

    // Precalculate common terms 

    // Sector specific duties
    switch (sector)
    {
        case SECT1:
        {
            break;
        }

        case SECT2:
        {
            break;
        }

        case SECT3:
        {
            break; 
        }

        case SECT4:
        {
           break; 
        }

        case SECT5:
        {
            break;
        }

        case SECT6:
        {
            break;
        } 

        default:
            duty->A = 0;
            duty->B = 0;
            duty->C = 0;
            break;
    }
}  


/* Calculates Clarke Transform (a,b,c -> alpha,beta)
    Parameters:
        *abcValues                 pointer to structure for a,b,c axis components
        *alphabetaValues           pointer to structure for alpha,beta axis components
*/
static inline void ClarkeTransform(struct_ABC *abcValues, struct_AlphaBeta *alphabetaValues)
{
    alphabetaValues->alpha = abcValues->a;
    alphabetaValues->beta = (abcValues->a * ONE_OVER_SQRT3) + (abcValues->b * TWO_OVER_SQRT3);
}


/* Calculates Inverse Clarke Transform (alpha,beta -> a,b,c)
    Parameters:
        *alphabetaValues           pointer to structure for alpha,beta axis components
        *abcValues                 pointer to structure for a,b,c axis components
    https://www.mathworks.com/help/sps/ref/inverseclarketransform.html
*/
static inline void InverseClarkeTransform(struct_AlphaBeta *alphabetaValues, struct_ABC *abcValues)
{
    abcValues->a = alphabetaValues->alpha;
    abcValues->b = (-alphabetaParam->alpha/2.0f + SQRT3_OVER_TWO * alphabetaParam->beta);
    abcValues->c = (-alphabetaParam->alpha/2.0f - SQRT3_OVER_TWO * alphabetaParam->beta);     
}


/* Calculates Park Transform (alpha,beta -> d,q)
    Parameters:
        *alphabetaValues           pointer to structure for alpha,beta axis components
        *dqValues                  pointer to structure for d,q axis components
        theta                      angle θ = ωt, where:
                                        θ is the angle between the α- and d-axes for the d-axis alignment or the angle between the α- and q-axes for the q-axis alignment. 
                                        It indicates the angular position of the rotating dq reference frame with respect to the α-axis.
                                        ω is the rotational speed of the d-q reference frame.
                                        t is the time, in seconds, from the initial alignment.
*/
static inline void ParkTransform(struct_AlphaBeta *alphabetaValues, struct_DQ *dqValues, float angle)
{
    float sinTheta;
    float cosTheta;
    CalcSinCos(&sinTheta, &cosTheta, angle)

    // assuming d-axis aligns with the α-axis initially
    dqValues->d = alphabetaValues->alpha * cosTheta + alphabetaValues->beta * sinTheta;
    dqValues->q = alphabetaValues->alpha * (-1) * sinTheta + alphabetaValues->beta * cosTheta;
}


/* Calculates Inverse Park Transform (d,q -> alpha,beta)
    Parameters:
        *alphabetaValues           pointer to structure for alpha,beta axis components
        *dqValues                  pointer to structure for d,q axis components
        theta                      angle θ = ωt, where:
                                        θ is the angle between the α- and d-axes for the d-axis alignment or the angle between the α- and q-axes for the q-axis alignment. 
                                        It indicates the angular position of the rotating dq reference frame with respect to the α-axis.
                                        ω is the rotational speed of the d-q reference frame.
                                        t is the time, in seconds, from the initial alignment.
*/
static inline void InverseParkTransform(struct_AlphaBeta *alphabetaValues, struct_DQ *dqValues, float angle)
{
    float sinTheta;
    float cosTheta;
    CalcSinCos(&sinTheta, &cosTheta, angle)

    // assuming d-axis aligns with the α-axis initially
    alphabetaValues->alpha = dqValues->d * cosTheta - dqValues->q * sinTheta;
    alphabetaValues->beta = dqValues->d * sinTheta + dqValues->q * cosTheta;   
}


/* Calculates Sine/Cosine values
    Parameters:
        *sinTheta           pointer to sine values for given angle
        *cosTheta           pointer to sine values for given angle
        theta               angle in radians
*/
void CalcSinCos(float *sinTheta, float *cosTheta, float angle)
{
   
    // Interpolate index to discrete value
    // y = y0 + (y1 - y0)*((x - x0)/(x1 - x0))
    float theta = angle;
    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, y0, y1, temp;
    
    // Software check to ensure  0 <= Angle < 2*PI
    if(theta <  0.0f) 
    {
        theta = theta + ANGLE_2PI; 
    }
    else if(theta >= ANGLE_2PI)
    {
        theta = theta - ANGLE_2PI; 
    }
    
    y0_Index = (uint32_t)((float)(theta/ANGLE_RES));
    
    if(y0_Index>=TABLE_SIZE)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
        temp = 0.0f;
    }
    else
    {
        y0_IndexNext = y0_Index + 1U;
        if(y0_IndexNext >= TABLE_SIZE )
        {
            y0_IndexNext = 0;
        }

        x0 = ((float)y0_Index * ANGLE_RES);  
    
    
        // Since below calculation is same for sine & cosine, we can do it once and reuse
	    temp = ((theta - x0)*ONE_OVER_ANGLE_RES);
    }
    
	// Calc Sine
    y0 = SinLookupTable[y0_Index];
    y1 = SinLookupTable[y0_IndexNext];     
    *sinTheta = y0 + ((y1 - y0)*temp);
	
    // Calc Cosine
    y0 = CosLookupTable[y0_Index];
    y1 = CosLookupTable[y0_IndexNext];
    *cosTheta = y0 + ((y1 - y0)*temp);
}


// Table containing 256 points for one Sine period with amplitude -1 to +1
static const float SinLookupTable[LUT_SIZE] = 
{
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f,
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f
};


// Table containing 256 points for one Cosine period with amplitude -1 to +1
static const float CosLookupTable[LUT_SIZE] = 
{
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f,
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f
};

