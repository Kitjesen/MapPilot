from __future__ import annotations


def test_slam_stack_factory_has_no_service_manager_dependency(monkeypatch):
    import runtime.service_manager as service_manager
    from lingtu.assembly.stacks.slam import slam

    def fail_get_service_manager():
        raise AssertionError("Blueprint stack must not manage processes")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_get_service_manager)

    slam("fastlio2", enable_visual_backup=False)
