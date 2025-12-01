"""
Logging configuration utilities.

Provides consistent logging setup across all modules.
"""

import logging
import sys
from typing import Optional
from pathlib import Path


# ANSI color codes for terminal output
class Colors:
    """ANSI color codes."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    GRAY = '\033[90m'


class ColoredFormatter(logging.Formatter):
    """Custom formatter with colors for console output."""
    
    COLORS = {
        'DEBUG': Colors.GRAY,
        'INFO': Colors.GREEN,
        'WARNING': Colors.YELLOW,
        'ERROR': Colors.RED,
        'CRITICAL': Colors.RED + Colors.BOLD,
    }
    
    def format(self, record):
        """Format log record with colors."""
        # Add color to level name
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = (
                f'{self.COLORS[levelname]}{levelname}{Colors.RESET}'
            )
        
        # Format the message
        result = super().format(record)
        
        # Reset levelname for subsequent formatters
        record.levelname = levelname
        
        return result


def setup_logging(
    name: str,
    level: str = 'INFO',
    log_file: Optional[str] = None,
    use_colors: bool = True
) -> logging.Logger:
    """
    Setup logging configuration.
    
    Args:
        name: Logger name
        level: Log level ('DEBUG', 'INFO', 'WARNING', 'ERROR')
        log_file: Optional log file path
        use_colors: Use colored output for console
    
    Returns:
        Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper()))
    
    # Remove existing handlers
    logger.handlers.clear()
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG)
    
    if use_colors and sys.stdout.isatty():
        console_format = (
            f'{Colors.GRAY}%(asctime)s{Colors.RESET} '
            f'%(levelname)s '
            f'{Colors.CYAN}[%(name)s]{Colors.RESET} '
            f'%(message)s'
        )
        console_formatter = ColoredFormatter(
            console_format,
            datefmt='%H:%M:%S'
        )
    else:
        console_format = '%(asctime)s %(levelname)-8s [%(name)s] %(message)s'
        console_formatter = logging.Formatter(
            console_format,
            datefmt='%H:%M:%S'
        )
    
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    # File handler (if specified)
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        
        file_format = (
            '%(asctime)s %(levelname)-8s [%(name)s] '
            '%(filename)s:%(lineno)d - %(message)s'
        )
        file_formatter = logging.Formatter(
            file_format,
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
    
    return logger


def get_ros_log_level(ros_level: str) -> str:
    """
    Convert ROS log level to Python logging level.
    
    Args:
        ros_level: ROS log level string
    
    Returns:
        Python logging level string
    """
    mapping = {
        'DEBUG': 'DEBUG',
        'INFO': 'INFO',
        'WARN': 'WARNING',
        'ERROR': 'ERROR',
        'FATAL': 'CRITICAL',
    }
    
    return mapping.get(ros_level.upper(), 'INFO')


def log_can_frame(logger: logging.Logger, direction: str, can_id: int, data: bytes):
    """
    Log CAN frame with consistent formatting.
    
    Args:
        logger: Logger instance
        direction: 'TX' or 'RX'
        can_id: CAN arbitration ID
        data: Frame data
    """
    data_hex = ' '.join(f'{b:02X}' for b in data)
    logger.debug(f'{direction} | ID: 0x{can_id:03X} | Data: {data_hex}')


def log_sdo_transaction(
    logger: logging.Logger,
    direction: str,
    index: int,
    subindex: int,
    data: Optional[bytes] = None,
    success: Optional[bool] = None,
    error_code: Optional[int] = None
):
    """
    Log SDO transaction with consistent formatting.
    
    Args:
        logger: Logger instance
        direction: 'TX' or 'RX'
        index: Object dictionary index
        subindex: Subindex
        data: Optional data bytes
        success: Optional success flag
        error_code: Optional error code
    """
    msg = f'{direction} SDO | 0x{index:04X}:0x{subindex:02X}'
    
    if data is not None:
        data_hex = ' '.join(f'{b:02X}' for b in data)
        msg += f' | Data: {data_hex}'
    
    if success is not None:
        if success:
            msg += ' | ✓'
        else:
            msg += ' | ✗'
            if error_code is not None:
                msg += f' (0x{error_code:08X})'
    
    logger.debug(msg)


class PerformanceLogger:
    """Logger for performance metrics."""
    
    def __init__(self, logger: logging.Logger):
        """
        Initialize performance logger.
        
        Args:
            logger: Base logger instance
        """
        self.logger = logger
        self.metrics = {}
    
    def log_metric(self, name: str, value: float, unit: str = ''):
        """
        Log a performance metric.
        
        Args:
            name: Metric name
            value: Metric value
            unit: Optional unit string
        """
        self.metrics[name] = value
        unit_str = f' {unit}' if unit else ''
        self.logger.debug(f'METRIC | {name}: {value:.3f}{unit_str}')
    
    def log_summary(self):
        """Log summary of all metrics."""
        if not self.metrics:
            return
        
        self.logger.info('Performance Metrics Summary:')
        for name, value in self.metrics.items():
            self.logger.info(f'  {name}: {value:.3f}')
    
    def reset(self):
        """Reset all metrics."""
        self.metrics.clear()
