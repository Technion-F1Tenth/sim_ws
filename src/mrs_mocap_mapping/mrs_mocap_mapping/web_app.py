"""
Flask REST API + static UI for the mocap map editor (rclpy-free).

All handlers go through `MapState`; validation errors come back as
400 {"error": "..."} so the UI can show them verbatim.
"""

import base64

from flask import Flask, jsonify, request, send_from_directory

from .map_state import MapState


def create_app(state: MapState, static_dir: str) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.errorhandler(ValueError)
    def _bad_request(err):
        return jsonify(error=str(err)), 400

    @app.errorhandler(KeyError)
    def _not_found(err):
        return jsonify(error=f'unknown id {err}'), 404

    @app.get('/')
    def index():
        return send_from_directory(static_dir, 'index.html')

    @app.get('/static/<path:name>')
    def static_files(name):
        return send_from_directory(static_dir, name)

    @app.get('/api/state')
    def get_state():
        return jsonify(state.snapshot())

    @app.get('/api/occupancy')
    def get_occupancy():
        version, spec, grid = state.occupancy()
        return jsonify(version=version, grid=spec.to_dict(),
                       data=base64.b64encode(grid.tobytes()).decode('ascii'))

    @app.post('/api/preview')
    def preview():
        return jsonify(state.build_obstacle(_body()).to_dict())

    @app.post('/api/obstacles')
    def add_obstacle():
        return jsonify(state.add_obstacle(_body()).to_dict()), 201

    @app.patch('/api/obstacles/<obstacle_id>')
    def update_obstacle(obstacle_id):
        return jsonify(state.update_obstacle(obstacle_id, _body()).to_dict())

    @app.delete('/api/obstacles/<obstacle_id>')
    def delete_obstacle(obstacle_id):
        state.delete_obstacle(obstacle_id)
        return jsonify(ok=True)

    @app.put('/api/grid')
    def put_grid():
        return jsonify(state.set_grid(_body()).to_dict())

    @app.post('/api/grid/fit')
    def fit_grid():
        body = _body()
        return jsonify(state.fit_grid(float(body.get('margin', 0.5)),
                                      body.get('resolution')).to_dict())

    @app.post('/api/save')
    def save():
        return jsonify(state.save())

    return app


def _body() -> dict:
    body = request.get_json(silent=True)
    if not isinstance(body, dict):
        raise ValueError('expected a JSON object body')
    return body
