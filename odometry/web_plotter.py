"""
web_plotter.py
--------------
Headless Matplotlib Plotter served via Flask.
"""

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import io
import threading
from flask import Flask, send_file

# Force headless backend (Must be before importing pyplot)
matplotlib.use('Agg')


class WebPlotter:
    def __init__(self, port=5001):
        self.port = port
        self.app = Flask(__name__)

        # Data Storage (Protected by lock)
        self.lock = threading.Lock()
        self.x_history = [0.0]
        self.y_history = [0.0]
        self.current_x = 0.0
        self.current_y = 0.0

        # Configure Routes
        self.add_routes()

        # Start Server in Background Thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        # print(f"📊 Plotter Server running at http://0.0.0.0:{self.port}")

    def add_routes(self):
        @self.app.route('/')
        def index():
            # Simple HTML that auto-refreshes the image every 500ms
            return """
            <html>
                <head><title>Robot Path</title></head>
                <body style="text-align:center; font-family:sans-serif;">
                    <h1>Live Odometry</h1>
                    <img src="/plot.png" id="plot" style="border:1px solid #333;"/>
                    <script>
                        setInterval(function() {
                            var img = document.getElementById('plot');
                            img.src = '/plot.png?rand=' + Math.random();
                        }, 500);
                    </script>
                </body>
            </html>
            """

        @self.app.route('/plot.png')
        def plot_png():
            # Generate the plot on-demand (does not block robot loop)
            return self.generate_image()

    def update(self, x, y):
        """
        Quickly stores data. Extremely fast, won't lag robot.
        """

        with self.lock:
            self.x_history.append(x)
            self.y_history.append(y)
            self.current_x = x
            self.current_y = y

    def generate_image(self):
        """
        Draws the plot to an in-memory image buffer.
        """

        # Create a new figure (thread-safe approach)
        fig, ax = plt.subplots(figsize=(6, 6))

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
        plt.close(fig)  # Cleanup memory
        output.seek(0)

        return send_file(output, mimetype='image/png')

    def run_server(self):
        # Run Flask without the reloader so it doesn't spawn extra threads
        self.app.run(host='0.0.0.0', port=self.port, debug=False, use_reloader=False)
