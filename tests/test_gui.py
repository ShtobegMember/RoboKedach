import pytest
from unittest.mock import MagicMock
from PyQt6.QtGui import QColor

# Mocks do not need PyQt event loops
from base_station.gui.hud_painter import HUDPainter

def test_battery_bar_color_logic():
    """Test that the battery bar correctly shifts color based on voltage."""
    painter_mock = MagicMock()
    
    # Simulate low voltage (9.5V is < 20% on a 3S LiPo mapping 9.0v - 12.6v)
    # Range is 3.6V. 20% is 9.0 + 0.72 = 9.72V
    HUDPainter.draw_battery_bar(painter_mock, voltage=9.5, current=1.0, screen_w=800, screen_h=600)
    
    # Grab the color passed to setBrush for the fill (the second setBrush call)
    brush_calls = painter_mock.setBrush.call_args_list
    fill_color = brush_calls[1][0][0]
    
    assert isinstance(fill_color, QColor)
    # Expected Red color for < 20%
    assert fill_color.red() == 255
    assert fill_color.green() == 0

    painter_mock.reset_mock()

    # Simulate healthy voltage (11.5V)
    HUDPainter.draw_battery_bar(painter_mock, voltage=11.5, current=1.0, screen_w=800, screen_h=600)
    
    brush_calls = painter_mock.setBrush.call_args_list
    fill_color = brush_calls[1][0][0]
    
    # Expected Green color for > 20%
    assert fill_color.red() == 0
    assert fill_color.green() == 255