#ifndef SAFE_INTERVAL_HANDLER_H
#define SAFE_INTERVAL_HANDLER_H

#include <vector>
#include <utility>
#include <algorithm>
#include <limits>
#include <cmath>

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

template <typename MPTraits>
class SafeIntervalHandler {
public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID VID;
    typedef typename MPTraits::Path Path;

    // Add DynamicAgentState struct
    struct DynamicAgentState {
        size_t index;
        bool isReverse;
        DynamicAgentState(size_t idx, bool rev) : index(idx), isReverse(rev) {}
    };

    // Represents a time interval with start and end times
    struct Interval {
        double startTime;
        double endTime;

        Interval(double start, double end) : startTime(start), endTime(end) {}

        bool contains(double time) const {
            return time >= startTime && time <= endTime;
        }

        bool operator<(const Interval &other) const {
            return startTime < other.startTime;
        }
    };

    SafeIntervalHandler(double timeResolution = 0.1)
        : m_timeResolution(timeResolution) {}

    DynamicAgentState calculateAgentState(double currentTick, double totalTicks) {
        int direction = static_cast<int>(floor(currentTick / totalTicks)) % 2;
        size_t index;
        
        if (direction == 0) {  // Forward
            index = static_cast<size_t>(fmod(currentTick, totalTicks));
            return DynamicAgentState(index, false);
        } else {  // Reverse
            index = static_cast<size_t>(totalTicks - 1 - fmod(currentTick, totalTicks));
            return DynamicAgentState(index, true);
        }
    }

    std::vector<Interval> findSafeIntervals(
        const std::vector<std::vector<CfgType>> &dynamicAgentPaths,
        const CfgType &robotCfg,
        double maxTime)
    {
        std::vector<Interval> safeIntervals;
        safeIntervals.emplace_back(0, maxTime);

        for (const auto &agentPath : dynamicAgentPaths) {
            std::vector<Interval> collisionIntervals =
                findCollisionIntervalsWithAgent(agentPath, robotCfg, maxTime);
            safeIntervals = subtractCollisionIntervals(safeIntervals, collisionIntervals);
        }

        return safeIntervals;
    }

    double findEarliestSafeArrival(
        const std::vector<Interval> &safeIntervals,
        double earliestPossibleArrival)
    {
        for (const auto &interval : safeIntervals) {
            if (interval.contains(earliestPossibleArrival)) {
                return earliestPossibleArrival;
            }
            if (interval.startTime > earliestPossibleArrival) {
                return interval.startTime;
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    bool isMotionSafe(
        const CfgType &start,
        const CfgType &end,
        double startTime,
        double duration,
        const std::vector<std::vector<CfgType>> &dynamicAgentPaths)
    {
        std::vector<CfgType> motionPoints = interpolateMotion(start, end, duration);

        double currentTime = startTime;
        for (const auto &point : motionPoints) {
            std::vector<Interval> safeIntervals =
                findSafeIntervals(dynamicAgentPaths, point, startTime + duration);

            bool isSafe = false;
            for (const auto &interval : safeIntervals) {
                if (interval.contains(currentTime)) {
                    isSafe = true;
                    break;
                }
            }

            if (!isSafe)
                return false;
            currentTime += m_timeResolution;
        }

        return true;
    }

    double GetTimeResolution() const {
        return m_timeResolution;
    }

private:
    double m_timeResolution;

    CfgType getAgentConfigurationAtTime(
        const std::vector<CfgType> &agentPath,
        double currentTick)
    {
        if (agentPath.empty()) {
            throw std::runtime_error("Empty agent path");
        }

        double totalTicks = agentPath.size() - 1;
        DynamicAgentState agentState = calculateAgentState(currentTick, totalTicks);
        size_t nextIndex = std::min(agentState.index + 1, agentPath.size() - 1);
        double fraction = currentTick / m_timeResolution - floor(currentTick / m_timeResolution);

        return agentPath[agentState.index] + (agentPath[nextIndex] - agentPath[agentState.index]) * fraction;
    }

    std::vector<Interval> findCollisionIntervalsWithAgent(
        const std::vector<CfgType> &agentPath,
        const CfgType &robotCfg,
        double maxTime)
    {
        std::vector<Interval> collisionIntervals;
        
        if (agentPath.empty()) {
            return collisionIntervals;
        }

        double currentTime = 0;
        bool inCollision = false;
        double collisionStart = 0;
        
        while (currentTime <= maxTime) {
            CfgType agentCfg = getAgentConfigurationAtTime(agentPath, currentTime / m_timeResolution);
            bool collisionNow = checkCollision(robotCfg, agentCfg);

            if (collisionNow && !inCollision) {
                inCollision = true;
                collisionStart = currentTime;
            } else if (!collisionNow && inCollision) {
                inCollision = false;
                collisionIntervals.emplace_back(collisionStart, currentTime);
            }

            currentTime += m_timeResolution;
        }

        if (inCollision) {
            collisionIntervals.emplace_back(collisionStart, currentTime);
        }

        return collisionIntervals;
    }

    std::vector<Interval> subtractCollisionIntervals(
        const std::vector<Interval> &safeIntervals,
        const std::vector<Interval> &collisionIntervals)
    {
        std::vector<Interval> result = safeIntervals;

        for (const auto &collision : collisionIntervals) {
            std::vector<Interval> newResult;

            for (const auto &safe : result) {
                if (collision.endTime <= safe.startTime ||
                    collision.startTime >= safe.endTime) {
                    newResult.push_back(safe);
                    continue;
                }

                if (collision.startTime > safe.startTime) {
                    newResult.emplace_back(safe.startTime, collision.startTime);
                }
                if (collision.endTime < safe.endTime) {
                    newResult.emplace_back(collision.endTime, safe.endTime);
                }
            }

            result = std::move(newResult);
        }

        return result;
    }

    std::vector<CfgType> interpolateMotion(
        const CfgType &start,
        const CfgType &end,
        double duration)
    {
        std::vector<CfgType> points;
        int numPoints = static_cast<int>(duration / m_timeResolution);

        for (int i = 0; i <= numPoints; i++) {
            double t = static_cast<double>(i) / numPoints;
            CfgType interpolated = start + (end - start) * t;
            points.push_back(interpolated);
        }

        return points;
    }

    bool checkCollision(const CfgType &robotCfg, const CfgType &agentCfg) {
        robotCfg.ConfigureRobot();
        auto rb_multibody = robotCfg.GetMultiBody();

        auto ag_multibody = agentCfg.GetMultiBody();
        ag_multibody->Configure(agentCfg);

        size_t rb_numbody = rb_multibody->NumFreeBody();
        size_t ag_numbody = ag_multibody->NumFreeBody();

        CDInfo cdInfo;
        CollisionDetectionMethod *m_cdMethod = new Rapid();

        for (size_t i = 0; i < rb_numbody; ++i) {
            for (size_t j = 0; j < ag_numbody; ++j) {
                if (m_cdMethod->IsInCollision(rb_multibody->GetFreeBody(i), 
                    ag_multibody->GetFreeBody(j), cdInfo)) {
                    delete m_cdMethod;
                    return true;
                }
            }
        }
        delete m_cdMethod;
        return false;
    }
};

#endif