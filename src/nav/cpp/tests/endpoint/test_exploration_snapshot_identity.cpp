#include <cassert>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

#include "traversability/exploration_snapshot_identity.hpp"

namespace {

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
  const std::string hash(64U, 'a');
  const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
      "map",
      "product-session-1234",
      "warehouse",
      "warehouse-lineage:v7",
      hash,
  });

  assert(identity.session_id == "product-session-1234");
  assert(identity.map_id == "warehouse");
  assert(identity.map_version == 7);
  assert(identity.artifact_hash == hash);
  assert(!identity.live);
}

void testMapRouteRejectsMissingMapId() {
  assert(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "product-session-1234",
        "",
        "warehouse-lineage:v7",
        std::string(64U, 'a'),
    });
  }));
}

void testMapRouteRejectsMissingProductSession() {
  assert(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "",
        "warehouse",
        "warehouse-lineage:v7",
        std::string(64U, 'a'),
    });
  }));
}

void testMapRouteRejectsInvalidMapVersionIds() {
  const std::string invalid_versions[] = {
      "", "v7", "lineage:v0", "lineage:v07", "lineage:v7tail", "lineage:v9223372036854775808",
  };
  for (const auto &version : invalid_versions) {
    assert(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "map",
          "product-session-1234",
          "warehouse",
          version,
          std::string(64U, 'a'),
      });
    }));
  }
}

void testMapRouteRejectsInvalidArtifactHashes() {
  const std::string invalid_hashes[] = {
      "",
      std::string(63U, 'a'),
      std::string(65U, 'a'),
      std::string(63U, 'a') + "g",
  };
  for (const auto &hash : invalid_hashes) {
    assert(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "map",
          "product-session-1234",
          "warehouse",
          "warehouse-lineage:v7",
          hash,
      });
    }));
  }
}

void testLiveRouteCarriesProductSessionIdentity() {
  const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
      "live",
      "product-session-1234",
      "ignored-map",
      "ignored-version",
      "ignored-hash",
  });
  assert(identity.session_id == "product-session-1234");
  assert(identity.map_id.empty());
  assert(identity.map_version == 0);
  assert(identity.artifact_hash.empty());
  assert(identity.live);
}

void testLiveRouteRejectsMissingOrInvalidProductSession() {
  const std::string invalid_sessions[] = {
      "",
      std::string(15U, 'a'),
      "product/session-1",
      std::string(129U, 'a'),
  };
  for (const std::string &product_session : invalid_sessions) {
    assert(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "live",
          product_session,
          "ignored-map",
          "ignored-version",
          "ignored-hash",
      });
    }));
  }
}

void testProductSessionLengthBoundaries() {
  for (const std::size_t length : {16U, 128U}) {
    const std::string product_session(length, 'a');
    const auto identity = lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "live",
        product_session,
        "ignored-map",
        "ignored-version",
        "ignored-hash",
    });
    assert(identity.session_id == product_session);
  }
}

void testMissingRouteFailsFast() {
  assert(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "",
        "product-session-1234",
        "ignored-map",
        "ignored-version",
        "ignored-hash",
    });
  }));
}

void testInvalidRouteFailsFast() {
  assert(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "auto",
        "product-session-1234",
        "warehouse",
        "warehouse-lineage:v7",
        std::string(64U, 'a'),
    });
  }));
}

void testMapRouteRejectsMalformedSourceNames() {
  const std::string invalid_map_ids[] = {" map", ".hidden", "-switch", "../warehouse", "a/b"};
  for (const auto &map_id : invalid_map_ids) {
    assert(throwsException([&] {
      (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
          "map",
          "product-session-1234",
          map_id,
          "warehouse-lineage:v7",
          std::string(64U, 'a'),
      });
    }));
  }

  assert(throwsException([] {
    (void)lingtu::nav::endpoint::resolveExplorationSnapshotIdentity({
        "map",
        "product/session",
        "warehouse",
        "warehouse-lineage:v7",
        std::string(64U, 'a'),
    });
  }));
}

}  // namespace

int main() {
  testMapRouteCarriesDeclaredMapIdentity();
  testMapRouteRejectsMissingMapId();
  testMapRouteRejectsMissingProductSession();
  testMapRouteRejectsInvalidMapVersionIds();
  testMapRouteRejectsInvalidArtifactHashes();
  testLiveRouteCarriesProductSessionIdentity();
  testLiveRouteRejectsMissingOrInvalidProductSession();
  testProductSessionLengthBoundaries();
  testMissingRouteFailsFast();
  testInvalidRouteFailsFast();
  testMapRouteRejectsMalformedSourceNames();
  std::cout << "test_exploration_snapshot_identity passed\n";
  return 0;
}
