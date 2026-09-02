#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "traversability/exploration_snapshot_identity.hpp"

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("exploration snapshot identity expectation failed");
  }
}

template <typename Function>
bool throwsException(Function &&function) {
  try {
    function();
  } catch (const std::exception &) {
    return true;
  }
  return false;
}

void testMapRouteCarriesDeclaredMapIdentity() {
  const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
      "map",
      "product-session-1234",
      "warehouse",
      "7",
  });

  require(identity.session_id == "product-session-1234");
  require(identity.map_id == "warehouse");
  require(identity.map_content_epoch == 7);
  require(!identity.live);
}

void testMapRouteRejectsMissingMapId() {
  require(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "product-session-1234",
        "",
        "7",
    });
  }));
}

void testMapRouteRejectsMissingProductSession() {
  require(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "",
        "warehouse",
        "7",
    });
  }));
}

void testMapRouteRejectsInvalidContentEpochs() {
  const std::string invalid_epochs[] = {
      "", "v7", "0", "07", "7tail", "9223372036854775808",
  };
  for (const auto &content_epoch : invalid_epochs) {
    require(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "map",
          "product-session-1234",
          "warehouse",
          content_epoch,
      });
    }));
  }
}

void testLiveRouteCarriesProductSessionIdentity() {
  const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
      "live",
      "product-session-1234",
      "ignored-map",
      "ignored-content-epoch",
  });
  require(identity.session_id == "product-session-1234");
  require(identity.map_id.empty());
  require(identity.map_content_epoch == 0);
  require(identity.live);
}

void testLiveRouteRejectsMissingOrInvalidProductSession() {
  const std::string invalid_sessions[] = {
      "",
      "-product-session",
      "product/session-1",
      std::string(64U, 'a'),
  };
  for (const std::string &product_session_id : invalid_sessions) {
    require(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "live",
          product_session_id,
          "ignored-map",
          "ignored-content-epoch",
      });
    }));
  }
}

void testProductSessionLengthBoundaries() {
  for (const std::size_t length : {1U, 63U}) {
    const std::string product_session_id(length, 'a');
    const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "live",
        product_session_id,
        "ignored-map",
        "ignored-version",
    });
    require(identity.session_id == product_session_id);
  }
}

void testMissingRouteFailsFast() {
  require(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "",
        "product-session-1234",
        "ignored-map",
        "ignored-version",
    });
  }));
}

void testInvalidRouteFailsFast() {
  require(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "auto",
        "product-session-1234",
        "warehouse",
        "7",
    });
  }));
}

void testMapRouteRejectsMalformedSourceNames() {
  const std::string invalid_map_ids[] = {" map", ".hidden", "-switch", "../warehouse", "a/b"};
  for (const auto &map_id : invalid_map_ids) {
    require(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "map",
          "product-session-1234",
          map_id,
          "7",
      });
    }));
  }

  require(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "product/session",
        "warehouse",
        "7",
    });
  }));
}

}  // namespace

int main() {
  testMapRouteCarriesDeclaredMapIdentity();
  testMapRouteRejectsMissingMapId();
  testMapRouteRejectsMissingProductSession();
  testMapRouteRejectsInvalidContentEpochs();
  testLiveRouteCarriesProductSessionIdentity();
  testLiveRouteRejectsMissingOrInvalidProductSession();
  testProductSessionLengthBoundaries();
  testMissingRouteFailsFast();
  testInvalidRouteFailsFast();
  testMapRouteRejectsMalformedSourceNames();
  std::cout << "test_exploration_snapshot_identity passed\n";
  return 0;
}
