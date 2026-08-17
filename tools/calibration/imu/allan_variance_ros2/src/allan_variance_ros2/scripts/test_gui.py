# library imports
import numpy as np
import pandas as pd

# create holoviews graph
hv_plot = hv.Points(df)

# display graph in browser
# a bokeh server is automatically started
bokeh_server = pn.Row(hv_plot).show(port=12345)

# stop the bokeh server (when needed)
bokeh_server.stop()
