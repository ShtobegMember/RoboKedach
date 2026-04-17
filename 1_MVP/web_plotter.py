"""
web_plotter.py - Real-time odometry plot served as PNG via Flask.
Uses cached rendering to avoid re-drawing when the robot hasn't moved.
"""

import io
import threading
from flask import Flask, send_file, make_response
import matplotlib.ticker as ticker

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


class WebPlotter:
    def __init__(self, port=5001):
        self.port = port
        self.app = Flask(__name__)

        # Path history and current position
        self.lock = threading.Lock()
        self.x_history = [0.0]
        self.y_history = [0.0]
        self.current_x = 0.0
        self.current_y = 0.0

        # Render cache: only re-draw when data changes
        self.needs_render = True
        self.cached_image = None

        self.add_routes()

        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def add_routes(self):
        @self.app.route('/')
        def index():
            html_content = """
            <html>
                <head><title>Robot Path</title></head>
                <body style="text-align:center; font-family:sans-serif;">
                    <h1>Live Odometry</h1>
                    <img src="/plot.png" id="plot" style="border:1px solid #333;"/>
                    <script>
                        setInterval(function() {
                            var img = document.getElementById('plot');
                            img.src = '/plot.png?rand=' + Math.random();
                        }, 1000);
                    </script>
                </body>
            </html>
            """
            response = make_response(html_content)
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            return response

        @self.app.route('/plot.png')
        def plot_png():
            response = self.generate_image()
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            return response

    def update(self, x, y):
        """Store new position and flag for re-render on next web request."""
        with self.lock:
            self.x_history.append(x)
            self.y_history.append(y)
            self.current_x = x
            self.current_y = y
            self.needs_render = True

    def generate_image(self):
        """Render the plot PNG. Returns cached image if robot hasn't moved."""
        with self.lock:
            if not self.needs_render and self.cached_image is not None:
                output = io.BytesIO(self.cached_image)
                return send_file(output, mimetype='image/png')

        fig = Figure(figsize=(6, 6))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)

        with self.lock:
            xs = list(self.x_history)
            ys = list(self.y_history)
            cx, cy = self.current_x, self.current_y

        ax.set_title("Robot Path (Real-Time)")
        ax.set_xlabel("Lateral (Y) [m]")
        ax.set_ylabel("Forward (X) [m]")
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
        ax.set_aspect('equal')

        # Plot path and current position marker
        ax.plot(ys, xs, 'bo-', markersize=4, label='Path')
        ax.plot(cy, cx, 'rx', markersize=8, markeredgewidth=2, label='Robot')
        ax.legend(loc='upper left')

        # Auto-zoom with margin
        margin = 1.0
        if len(xs) > 1:
            ax.set_xlim(min(ys) - margin, max(ys) + margin)
            ax.set_ylim(min(xs) - margin, max(xs) + margin)
        else:
            ax.set_xlim(-margin, margin)
            ax.set_ylim(-margin, margin)

        output = io.BytesIO()
        fig.savefig(output, format='png')
        output.seek(0)

        with self.lock:
            self.cached_image = output.getvalue()
            self.needs_render = False

        return send_file(output, mimetype='image/png')

    def run_server(self):
        self.app.run(host='0.0.0.0', port=self.port, debug=False, use_reloader=False)
