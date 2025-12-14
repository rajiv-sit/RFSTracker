#pragma once

#include "association/HungarianSolver.hpp"
#include "config/TrackerConfig.hpp"
#include "filters/IrfsFilter.hpp"
#include "representations/RepresentationFactory.hpp"
#include "simulation/MeasurementSet.hpp"

#include <memory>
#include <vector>

namespace rfs {

class MbFilter : public IrfsFilter {
public:
  explicit MbFilter(RepresentationType representationType,
                    const TrackerConfig *config);

  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
  EstimatorOutput_t estimate() const override;

protected:
  struct BernoulliHypothesis {
    std::unique_ptr<IRepresentation> representation;
    double existence = 0.5;
    int id = 0;
  };

  const std::vector<BernoulliHypothesis> &hypotheses() const;

  std::unique_ptr<IRepresentation> createEmptyRepresentation() const;
  void pruneHypotheses();
  double detectionProbability() const;

  std::vector<BernoulliHypothesis> hypotheses_;
  RepresentationType representationType_;
  const TrackerConfig *config_;
  HungarianSolver solver_;
  int nextHypothesisId_ = 1;
};

} // namespace rfs
