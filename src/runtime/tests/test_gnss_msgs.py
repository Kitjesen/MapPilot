from __future__ import annotations

from runtime.msgs.gnss import GnssFix, GnssFixType, GnssOdom


def test_gnss_fix_ros_dict_shape() -> None:
    fix = GnssFix(
        lat=30.0,
        lon=120.0,
        alt=12.5,
        fix_type=GnssFixType.RTK_FIXED,
        covariance=(0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.04),
        ts=1.25,
        frame_id="gnss",
    )

    ros = fix.to_ros_dict()

    assert set(ros) == {
        "header",
        "status",
        "latitude",
        "longitude",
        "altitude",
        "position_covariance",
        "position_covariance_type",
    }
    assert ros["header"] == {"stamp": {"sec": 1, "nanosec": 250_000_000}, "frame_id": "gnss"}
    assert ros["status"] == {"status": 2, "service": 1}
    assert ros["position_covariance_type"] == 3


def test_gnss_odom_ros_dict_shape() -> None:
    odom = GnssOdom(
        east=1.0,
        north=2.0,
        up=3.0,
        ve=0.1,
        vn=0.2,
        vu=0.3,
        cov_e=0.01,
        cov_n=0.02,
        cov_u=0.03,
        ts=2.5,
        frame_id="map",
    )

    ros = odom.to_ros_dict()

    assert set(ros) == {"header", "child_frame_id", "pose", "twist"}
    assert ros["header"] == {"stamp": {"sec": 2, "nanosec": 500_000_000}, "frame_id": "map"}
    assert ros["pose"]["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert ros["pose"]["covariance"][0] == 0.01
    assert ros["pose"]["covariance"][7] == 0.02
    assert ros["pose"]["covariance"][14] == 0.03
    assert ros["twist"]["twist"]["linear"] == {"x": 0.1, "y": 0.2, "z": 0.3}
