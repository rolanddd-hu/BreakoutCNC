#pragma once
#include <memory>
#include <string>
#include <vector>

namespace rnr::core {

// Reads "v0.3.1", "0.3" and similar into its numbers. Returns false for anything else, such as "v0.4-beta".
bool parseVersion(const std::string& text, std::vector<int>& out);

// Negative when a < b, 0 when equal, positive when a > b. Missing numbers count as 0, so 0.3 == 0.3.0.
int compareVersions(const std::vector<int>& a, const std::vector<int>& b);

// The "tag_name" value of a GitHub release response, or empty if there is none.
std::string latestTagFromReleaseJson(const std::string& json);

// The version in a GitHub release response (without the "v") if it is newer than `current`, otherwise empty.
std::string newerVersionInReleaseJson(const std::string& json, const std::string& current);

// Asks GitHub once for the latest release, on its own thread. Any failure (no internet connection, a proxy, GitHub
// not reachable, an unexpected answer) simply leaves availableVersion() empty.
class UpdateChecker {
public:
    // Returns at once. Only the first call does anything.
    void start(const std::string& current_version);

    // The newer version to show, or empty. Cheap enough to call on every frame.
    std::string availableVersion() const;

private:
    struct State;
    std::shared_ptr<State> state_; // Shared with the thread, which is not waited for: closing the app must not hang
};

} // namespace rnr::core
