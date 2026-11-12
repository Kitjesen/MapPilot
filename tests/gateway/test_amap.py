import json
from urllib.parse import parse_qs, urlsplit

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.amap import register_amap_routes


@pytest.fixture
def setup(tmp_path, monkeypatch):
    config = tmp_path / 'amap.json'
    monkeypatch.setenv('LINGTU_AMAP_CONFIG', str(config))
    monkeypatch.delenv('LINGTU_AMAP_KEY', raising=False)
    monkeypatch.delenv('LINGTU_AMAP_SECURITY_CODE', raising=False)
    app = FastAPI()
    register_amap_routes(app)
    with TestClient(app) as client:
        yield client, config


def test_config_updates_without_restart_and_never_exposes_security_code(setup):
    client, config = setup
    assert client.get('/api/v1/geography/map-config').json()['state'] == 'missing'
    config.write_text(json.dumps({'key': 'public', 'security_code': ''}))
    assert client.get('/api/v1/geography/map-config').json()['state'] == 'incomplete'
    config.write_text(json.dumps({'key': 'public', 'security_code': 'private-code'}))
    response = client.get('/api/v1/geography/map-config')
    assert response.json() == {'state': 'configured', 'key': 'public', 'service_host': '/_AMapService'}
    assert 'private-code' not in response.text
    assert response.headers['cache-control'] == 'no-store'
    config.write_text('{broken')
    assert client.get('/api/v1/geography/map-config').json()['state'] == 'invalid'


def test_unconfigured_proxy_does_not_request_upstream(setup, monkeypatch):
    client, _ = setup
    monkeypatch.setattr('gateway.routes.amap.http.client.HTTPSConnection', lambda *a, **k: pytest.fail('unexpected network'))
    assert client.get('/_AMapService/v3/assistant/coordinate/convert').status_code == 503


@pytest.mark.parametrize(('path', 'host'), [
    ('v4/map/styles', 'webapi.amap.com'),
    ('v3/assistant/coordinate/convert', 'restapi.amap.com'),
])
def test_proxy_pins_host_and_injects_only_server_credentials(setup, monkeypatch, path, host):
    client, config = setup
    config.write_text(json.dumps({'key': 'public', 'security_code': 'private-code'}))
    requests = []
    closed = []

    class Connection:
        def __init__(self, target, timeout):
            assert target == host and timeout == 10
        def request(self, method, url):
            requests.append((method, url))
        def getresponse(self):
            return self
        status = 200
        def read(self, limit):
            return b'{"status":"1"}'
        def getheader(self, name, default):
            return 'application/json'
        def close(self):
            closed.append(True)

    monkeypatch.setattr('gateway.routes.amap.http.client.HTTPSConnection', Connection)
    result = client.get(f'/_AMapService/{path}?key=wrong&jscode=wrong&locations=116,39', headers={'Cookie': 'private-session=1'})
    assert result.json() == {'status': '1'}
    assert len(requests) == 1 and closed == [True]
    method, url = requests[0]
    assert method == 'GET' and urlsplit(url).path == '/' + path
    assert parse_qs(urlsplit(url).query) == {'key': ['public'], 'jscode': ['private-code'], 'locations': ['116,39']}
    assert 'private-code' not in result.text and 'set-cookie' not in result.headers


def test_proxy_rejects_arbitrary_paths_and_hides_upstream_errors(setup, monkeypatch):
    client, config = setup
    config.write_text(json.dumps({'key': 'public', 'security_code': 'private-code'}))
    assert client.get('/_AMapService/https://other.example/x').status_code == 400

    class Connection:
        def __init__(self, *args, **kwargs): pass
        def request(self, *args): raise OSError('URL contains private-code')
        def close(self): pass
    monkeypatch.setattr('gateway.routes.amap.http.client.HTTPSConnection', Connection)
    response = client.get('/_AMapService/v3/assistant/coordinate/convert')
    assert response.status_code == 502
    assert 'private-code' not in response.text
