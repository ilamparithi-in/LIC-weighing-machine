#pragma once

#include <Arduino.h>
#include <cmath>

// ---------------------- CONVERSION CONSTANTS -----------------
constexpr float GRAMS_PER_OUNCE = 28.349523f;
constexpr float GRAMS_PER_POUND = 453.59237f;

// ---------------------- UNITS OF MEASURE --------------------
enum WeightUnit {
  UNIT_G = 0,                    // Grams (e.g., 607 g)
  UNIT_KG = 1,                   // Kilograms (e.g., 0.607 kg)
  UNIT_OZ = 2,                   // Ounces (e.g., 21.41 oz)
  UNIT_LB = 3                    // Pounds (e.g., 1.338 lb)
};

inline const char* getUnitSymbol(WeightUnit unit) {
  switch (unit) {
    case UNIT_KG: return "kg";
    case UNIT_OZ: return "oz";
    case UNIT_LB: return "lb";
    case UNIT_G:
    default:      return "g";
  }
}

inline const char* getUnitName(WeightUnit unit) {
  switch (unit) {
    case UNIT_KG: return "Kilograms";
    case UNIT_OZ: return "Ounces";
    case UNIT_LB: return "Pounds";
    case UNIT_G:
    default:      return "Grams";
  }
}

/**
 * Formats weight value into a string including the unit symbol:
 * - g:  integer resolution (e.g., "  607 g")
 * - kg: 3 decimals for 1g resolution (e.g., "0.607 kg")
 * - oz: 2 decimals for ~0.035 oz resolution (e.g., "21.41 oz")
 * - lb: 3 decimals for ~0.0022 lb resolution (e.g., "1.338 lb")
 */
inline void formatWeightString(float grams, WeightUnit unit, char* outBuf, size_t maxLen) {
  switch (unit) {
    case UNIT_KG: {
      float kg = grams / 1000.0f;
      if (fabsf(kg) < 0.0005f) kg = 0.0f;
      snprintf(outBuf, maxLen, "%.3f kg", kg);
      break;
    }
    case UNIT_OZ: {
      float oz = grams / GRAMS_PER_OUNCE;
      if (fabsf(oz) < 0.005f) oz = 0.0f;
      snprintf(outBuf, maxLen, "%.2f oz", oz);
      break;
    }
    case UNIT_LB: {
      float lb = grams / GRAMS_PER_POUND;
      if (fabsf(lb) < 0.0005f) lb = 0.0f;
      snprintf(outBuf, maxLen, "%.3f lb", lb);
      break;
    }
    case UNIT_G:
    default: {
      long g = lround(grams);
      snprintf(outBuf, maxLen, "%5ld g", g);
      break;
    }
  }
}

/**
 * Formats just the numeric weight value without unit or leading padding.
 */
inline void formatWeightNumber(float grams, WeightUnit unit, char* outBuf, size_t maxLen) {
  switch (unit) {
    case UNIT_KG: {
      float kg = grams / 1000.0f;
      if (fabsf(kg) < 0.0005f) kg = 0.0f;
      snprintf(outBuf, maxLen, "%.3f", kg);
      break;
    }
    case UNIT_OZ: {
      float oz = grams / GRAMS_PER_OUNCE;
      if (fabsf(oz) < 0.005f) oz = 0.0f;
      snprintf(outBuf, maxLen, "%.2f", oz);
      break;
    }
    case UNIT_LB: {
      float lb = grams / GRAMS_PER_POUND;
      if (fabsf(lb) < 0.0005f) lb = 0.0f;
      snprintf(outBuf, maxLen, "%.3f", lb);
      break;
    }
    case UNIT_G:
    default: {
      long g = lround(grams);
      snprintf(outBuf, maxLen, "%ld", g);
      break;
    }
  }
}
