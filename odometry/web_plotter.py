"""
web_plotter.py
--------------
Headless Matplotlib Plotter served via Flask.
Uses Object-Oriented Matplotlib and Smart Caching to ensure 
rendering only happens AFTER a movement finishes.
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

        # Data Storage
        self.lock = threading.Lock()
        self.x_history = [0.0]
        self.y_history = [0.0]
        self.current_x = 0.0
        self.current_y = 0.0

        # --- NEW: Smart Caching Variables ---
        self.needs_render = True
        self.cached_image = None
        # ------------------------------------

        self.add_routes()

        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def add_routes(self):
        @self.app.route('/')
        def index():
            # You can safely put this back to 500ms now if you want it to feel 
            # more responsive, because redundant requests cost zero CPU!
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
        """
        Stores data and trips the flag so the next web request renders the new path.
        This is ONLY called by main.py after the motors stop moving.
        """

        with self.lock:
            self.x_history.append(x)
            self.y_history.append(y)
            self.current_x = x
            self.current_y = y
            
            # --- NEW: Tell the server the data changed ---
            self.needs_render = True

    def generate_image(self):
        """
        Draws the plot. Only runs Matplotlib if the robot actually moved.
        """
        
        # --- NEW: Check the Cache first! ---
        with self.lock:
            if not self.needs_render and self.cached_image is not None:
                # The robot hasn't moved. Instantly return the old picture.
                # This uses almost ZERO CPU.
                output = io.BytesIO(self.cached_image)
                return send_file(output, mimetype='image/png')
        # -----------------------------------

        # If we reach here, it means the robot moved. Do the heavy rendering.
        fig = Figure(figsize=(6, 6))
        FigureCanvasAgg(fig) 
        ax = fig.add_subplot(111)

        with self.lock:
            xs = list(self.x_history)
            ys = list(self.y_history)
            cx, cy = self.current_x, self.current_y

        # Plot Config
        ax.set_title("Robot Path (Real-Time)")
        ax.set_xlabel("Lateral (Y) [m]")
        ax.set_ylabel("Forward (X) [m]")
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
        ax.set_aspect('equal')

        # Draw Path and Head
        ax.plot(ys, xs, 'bo-', markersize=4, label='Path')
        ax.plot(cy, cx, 'rx', markersize=8, markeredgewidth=2, label='Robot')
        ax.legend(loc='upper left')

        # Dynamic Zoom
        margin = 1.0
        if len(xs) > 1:
            ax.set_xlim(min(ys) - margin, max(ys) + margin)
            ax.set_ylim(min(xs) - margin, max(xs) + margin)
        else:
            ax.set_xlim(-margin, margin)
            ax.set_ylim(-margin, margin)

        # Save to buffer
        output = io.BytesIO()
        fig.savefig(output, format='png')
        output.seek(0)
        
        # --- NEW: Save to cache and reset flag ---
        with self.lock:
            self.cached_image = output.getvalue()
            self.needs_render = False

        return send_file(output, mimetype='image/png')

    def run_server(self):
        self.app.run(host='0.0.0.0', port=self.port, debug=False, use_reloader=False)
