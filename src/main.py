import logging 

logging.debug("Initialze the data filter")
data_filter = DataFilter()
for _ in range(100):
    logging.debug("Processing data batch")
    data_filter.process_batch()