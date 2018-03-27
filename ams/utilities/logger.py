#!/usr/bin/env python
# coding: utf-8

from logging import config, getLogger
from pprint import PrettyPrinter


config.dictConfig({
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "simple": {
            "format": "%(name)s %(asctime)s %(levelname)s pid %(process)d line %(lineno)d in %(funcName)s: %(message)s"
        },
    },
    "handlers": {
        "ams_console": {
            "class": "logging.StreamHandler",
            "level": "WARNING",
            "formatter": "simple",
            "stream": "ext://sys.stdout"},
        "ams_error_file": {
            "class": "logging.FileHandler",
            "level": "ERROR",
            "formatter": "simple",
            "filename": "./error.log",
            "encoding": "utf8"},
        "ams_warning_file": {
            "class": "logging.FileHandler",
            "level": "WARNING",
            "formatter": "simple",
            "filename": "./warning.log",
            "encoding": "utf8"},
        "ams_summary_file": {
            "class": "logging.handlers.RotatingFileHandler",
            "level": "INFO",
            "formatter": "simple",
            "filename": "./summary.log",
            "maxBytes": 10485760,
            "backupCount": 2,
            "encoding": "utf8"},
        "ams_http": {
            "class": "logging.handlers.HTTPHandler",
            "level": "WARNING",
            "formatter": "simple",
            "host": "localhost",
            "url": "localhost",
            "method": "post"}},
    "root": {
        "level": "DEBUG",
    },
    "loggers": {
        "ams": {
            "handlers": ["ams_console", "ams_warning_file", "ams_error_file", "ams_summary_file"],
            "qualname": "ams",
            "propagate": True}}})

logger = getLogger("ams")
logger.pp = PrettyPrinter(indent=2).pprint
