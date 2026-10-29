// persist_policy.cpp - see persist_policy.h.
#include "persist_policy.h"

namespace ot {

const char* loadStatusName(LoadStatus s) {
  switch (s) {
    case LoadStatus::Ok: return "ok";
    case LoadStatus::Missing: return "missing";
    case LoadStatus::BadSchema: return "bad_schema";
    case LoadStatus::BadValue: return "bad_value";
  }
  return "?";
}

const char* persistActionName(PersistAction a) {
  switch (a) {
    case PersistAction::None: return "none";
    case PersistAction::Commit: return "commit";
    case PersistAction::Defer: return "defer";
  }
  return "?";
}

uint8_t unitsByte(Units u) { return static_cast<uint8_t>(u); }

UnitsLoad validateUnits(bool hasSchema, uint8_t schema, bool hasUnits, uint8_t units, Units dflt) {
  UnitsLoad r;
  r.units = dflt;
  if (!hasSchema || !hasUnits) {
    r.status = LoadStatus::Missing;
  } else if (schema != NVS_SCHEMA) {
    r.status = LoadStatus::BadSchema;
  } else if (units == NVS_UNITS_MI) {
    r.units = Units::Mi;
    r.status = LoadStatus::Ok;
  } else if (units == NVS_UNITS_KM) {
    r.units = Units::Km;
    r.status = LoadStatus::Ok;
  } else {
    r.status = LoadStatus::BadValue;
  }
  return r;
}

PersistPolicy::PersistPolicy(const PersistConfig& cfg) : cfg_(cfg) {}

void PersistPolicy::begin(Units stored) {
  stored_ = stored;
  hasToggle_ = false;
  lastToggle_ = 0;
  hasWrite_ = false;
  lastWrite_ = 0;
  deferReported_ = false;
}

void PersistPolicy::onToggle(int64_t t) {
  if (!hasToggle_ || t > lastToggle_) lastToggle_ = t;  // never moves backwards
  hasToggle_ = true;
  deferReported_ = false;  // a new pending change: report a deferral again
}

PersistAction PersistPolicy::decide(int64_t now, Units current, bool hasPulse, int64_t lastPulseUs, bool holdActive) {
  if (current == stored_) {  // nothing (net) to write
    deferReported_ = false;
    return PersistAction::None;
  }
  if (hasToggle_ && now - lastToggle_ < cfg_.settleUs) return PersistAction::None;
  if (hasWrite_ && now - lastWrite_ < cfg_.minIntervalUs) return PersistAction::None;
  const bool moving = hasPulse && now - lastPulseUs <= cfg_.stopUs;
  if (moving || holdActive) {
    if (deferReported_) return PersistAction::None;
    deferReported_ = true;
    return PersistAction::Defer;
  }
  return PersistAction::Commit;
}

void PersistPolicy::written(int64_t now, Units value, bool ok) {
  hasWrite_ = true;
  lastWrite_ = now;
  deferReported_ = false;
  if (ok) stored_ = value;
}

}  // namespace ot
