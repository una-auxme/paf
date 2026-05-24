"""Utilities for consistent structured logging across agent modules."""

from __future__ import annotations

import json
import logging
import sys
from collections.abc import Mapping
from datetime import UTC, datetime
from typing import Any


class JsonLogFormatter(logging.Formatter):
    """Format log records as JSON objects with stable top-level fields."""

    def format(self, record: logging.LogRecord) -> str:
        """Convert a log record into a JSON string."""
        payload: dict[str, Any] = {
            "timestamp": datetime.now(UTC).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)

        extra_context = getattr(record, "context", None)
        if isinstance(extra_context, Mapping):
            payload["context"] = dict(extra_context)

        return json.dumps(payload, ensure_ascii=False)


class ContextLoggerAdapter(logging.LoggerAdapter):
    """Attach stable context fields to each emitted log record."""

    def process(self, msg: str, kwargs: dict[str, Any]) -> tuple[str, dict[str, Any]]:
        """Inject adapter context into the logging kwargs."""
        extra = kwargs.setdefault("extra", {})
        context = dict(extra.get("context", {}))
        context.update(self.extra)
        extra["context"] = context
        return msg, kwargs


def configure_logging(
    *,
    level: int = logging.INFO,
    json_logs: bool = False,
    logger_name: str | None = None,
) -> logging.Logger:
    """Configure and return a stream logger with project defaults.

    Args:
        level: Minimum emitted log level.
        json_logs: Emit JSON log entries when set to True.
        logger_name: Optional logger name. Uses root logger when omitted.

    Returns:
        Configured logger instance.
    """
    logger = logging.getLogger(logger_name)
    logger.setLevel(level)
    logger.handlers.clear()
    logger.propagate = False

    handler = logging.StreamHandler(sys.stdout)
    if json_logs:
        handler.setFormatter(JsonLogFormatter())
    else:
        handler.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)s [%(name)s] %(message)s",
                "%Y-%m-%dT%H:%M:%S%z",
            )
        )

    logger.addHandler(handler)
    return logger


def with_log_context(logger: logging.Logger, **context: Any) -> ContextLoggerAdapter:
    """Return a logger adapter that injects stable context keys.

    Args:
        logger: Base logger to wrap.
        **context: Context values to include in every log record.

    Returns:
        Logger adapter with context attached.
    """
    return ContextLoggerAdapter(logger, context)
