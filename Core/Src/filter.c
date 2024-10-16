// Filter Code
// FILTER_ALPHA  0.1 // Smoothing factor (0 < alpha < 1)

float LowPassFilter(float previous_value, float new_value,float filter_alpha){
    return (filter_alpha * new_value) + ((1.0 - filter_alpha) * previous_value);
}