#pragma once

typedef struct CostFunctionWeight{
    double A;
    double B;
    double C;
    double D;
    double E;
}CostFunctionWeight;

const CostFunctionWeight CF  = {10, 10, 0.1, -10, -1};