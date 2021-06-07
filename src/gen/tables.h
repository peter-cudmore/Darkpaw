typedef struct {
    int pwm;
    float low;
    float high;
    float value;
} AngleLookupTableItem;

extern AngleLookupTableItem BodyJoint[];
extern AngleLookupTableItem RadialJoint[];
extern AngleLookupTableItem HeightJoint[];
