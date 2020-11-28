// SPDX-License-Identifier: GPL-2.0-only
//
// GPIO chip drivers in userspace
//
// This module provides a special character device that userspace programs can
// open and use to register new "GPIO chip" devices whose main implementation
// lives in the userspace program.
//
// The intended purpose of this is for using various development-oriented,
// multi-function devices that outsource GPIO and possibly other functions to
// another system, often over a USB serial protocol. Examples of such devices
// include the Dangerous Prototypes Bus Pirate (in its "bitbang" mode) and
// the aux pins of Excamera Labs SPIDriver, both of which are used indirectly
// via a serial protocol over USB and involve a non-trivial wire protocol
// that's more appropriate to implement in userspace.
//
// GPIO chip drivers in userspace are not suitable for production platform
// use, but may be useful during development of an application that uses
// GPIOs via the userspace character device interface in order to work with
// external hardware on a general-purpose workstation before a full development
// platform is available.
//
// Copyright (C) 2020 Martin Atkins

#define DRV_NAME "gpio-user"
#define pr_fmt(fmt) DRV_NAME ": " fmt

#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/string.h>

static char gpio_user_header_magic[12] = { 'U', 'S', 'E', 'R', 'G', 'P',
					   'I', 'O', 0,	  0,   0,   0 };
static char *gpio_user_default_label = "gpio-user";
static struct miscdevice gpio_user_miscdevice; // initialized below

struct gpio_user_init_message {
	char magic[12]; // user value must match gpio_user_header_magic
	char label[12]; // null-terminated label string
	u32 ngpio; // number of GPIO signals the process is presenting
};

/*
 * Our message types are split into two broad categories: requests and events.
 * Requests are messages from kernel to userspace that the userspace program
 * is expected to eventually respnd to. Events are one-off messages where no
 * response is expected, which includes the response messages themselves and
 * some other special signals.
 */
#define GPIO_USER_MSG_PREPARE_LINE 1
#define GPIO_USER_MSG_FREE_LINE 2
#define GPIO_USER_MSG_GET_DIR 3
#define GPIO_USER_MSG_SET_DIR 4
#define GPIO_USER_MSG_SET_CONFIG 5
#define GPIO_USER_MSG_GET 6
#define GPIO_USER_MSG_SET 7
#define GPIO_USER_MSG_EVENT_MASK 0x80000000
#define GPIO_USER_MSG_EVENT(base) (msg | GPIO_USER_MSG_EVENT_MASK)
#define GPIO_USER_MSG_IS_EVENT(msg) ((msg & GPIO_USER_MSG_EVENT_MASK) != 0)
#define GPIO_USER_MSG_IS_REQUEST(msg) ((msg & GPIO_USER_MSG_EVENT_MASK) == 0)
#define GPIO_USER_MSG_RESPONSE GPIO_USER_MSG_EVENT(1)
#define GPIO_USER_MSG_CANCEL GPIO_USER_MSG_EVENT(2)

struct gpio_user_msg_body_offset {
	u32 offset;
};

struct gpio_user_msg_body_offset_val {
	u32 offset;
	u32 value;
};

struct gpio_user_msg_body_direction {
	u32 offset;
	u16 value;
	u16 is_output;
};

struct gpio_user_msg {
	u32 msg_type;
	u32 msg_id;
	u8 body[]; // body type depends on the message type
};

/*
 * gpio_user_session is a session between a userspace "driver" program and
 * the gpio-user kernel driver, started by opening the character device.
 *
 * This is the private_data associated with all open files from our device,
 * and also with each gpio_chip we register. Its lifetime exactly matches
 * the lifetime of the file it belongs to.
 * 
 * The purpose of the gpio-user driver is to proxy messages back and forth
 * between the userspace driver (represented by "file") and the kernelspace
 * GPIO chip (represented by "chip").
 */
struct gpio_user_session {
	// All operations on a session must hold this lock. Operations on a
	// particular session are always serialized.
	struct mutex lock;

	// The virtual GPIO chip device associated with this session. This
	// is initialized only after the userspace program has written the
	// initialization message to the file in order to provide the
	// chip configuration.
	//
	// Its uninitialized state is recognized by the "label" field being
	// NULL. Once initialized, "label" always points to a valid string.
	struct gpio_chip chip;

	// The file handle we use as the bidirectional message channel between
	// this driver and the userspace program. file->private_data always
	// points back to the same session.
	struct file *file;

	// If the chip has a custom label (different than default_label)
	// then this field is the backing storage for chip.label. Otherwise,
	// chip.label points directly at default_label and this field is
	// invalid. Use chip.label instead of accessing this directly.
	char _label[12];
};

inline int gpio_user_session_active(struct gpio_user_session *sess)
{
	return sess->chip.label != NULL;
}

size_t gpio_user_session_msg_size(struct gpio_user_session *sess, u32 ty)
{
	size_t base = sizeof(struct gpio_user_msg);
	switch (ty) {
	case GPIO_USER_MSG_PREPARE_LINE:
	case GPIO_USER_MSG_FREE_LINE:
	case GPIO_USER_MSG_GET_DIR:
		return base + sizeof(struct gpio_user_msg_body_offset);
	case GPIO_USER_MSG_SET_DIR:
		return base + sizeof(struct gpio_user_msg_body_direction);
	case GPIO_USER_MSG_SET_CONFIG:
		return base + sizeof(struct gpio_user_msg_body_offset_val);
	case GPIO_USER_MSG_SET:
	case GPIO_USER_MSG_GET:
		// Body is a bitmask with one bit per GPIO
		return base + (sess->chip.ngpio / 8);
	default:
		return base;
	}
}

size_t gpio_user_session_msg_resp_size(struct gpio_user_session *sess, u32 ty)
{
	size_t base = sizeof(struct gpio_user_msg);
	switch (ty) {
	case GPIO_USER_MSG_PREPARE_LINE:
	case GPIO_USER_MSG_FREE_LINE:
	case GPIO_USER_MSG_SET_DIR:
	case GPIO_USER_MSG_SET_CONFIG:
	case GPIO_USER_MSG_SET:
		return base;
	case GPIO_USER_MSG_GET_DIR:
		return base + sizeof(struct gpio_user_msg_body_direction);
	case GPIO_USER_MSG_GET:
		// Body is a bitmask with one bit per GPIO
		return base + (sess->chip.ngpio / 8);
	default:
		return base;
	}
}

/*
 * gpio_user_alloc_message allocates a buffer big enough to contain a message
 * of the given type and populates its msg_type and msg_id fields.
 *
 * If the message type has an associated body then the caller must populate
 * the remainder of the buffer with data of a suitable shape to make the
 * message valid.
 */
struct gpio_user_msg *gpio_user_alloc_msg(struct gpio_user_session *sess,
					  u32 ty, u32 id)
{
	size_t size = gpio_user_session_msg_size(sess, ty);
	struct gpio_user_msg *ret = kzalloc(size, GFP_KERNEL);
	if (ret == NULL) {
		return NULL;
	}
	ret->msg_type = ty;
	ret->msg_id = id;
	return ret;
}

/*
 * gpio_user_open is the "open" implementation for our character device, and
 * it establishes an uninitialized session which the user process must then
 * activate by writing an initialization message to it.
 */
int gpio_user_open(struct inode *in, struct file *f)
{
	struct gpio_user_session *sess;

	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;

	mutex_init(&sess->lock);

	sess->file = f;
	f->private_data = sess;
	return 0;
}

int gpio_user_release(struct inode *in, struct file *f)
{
	struct gpio_user_session *sess = f->private_data;
	if (sess == NULL) {
		return 0;
	}
	if (gpio_user_session_active(sess)) {
		gpiochip_remove(&sess->chip);
	}
	kfree(sess);
	return 0;
}

/*
 * gpio_user_read_early deals with a read call for an uninitialized session.
 * In that case, the response is a message containing the "magic number" for
 * our protocol, so that callers can recognize that they've opened an
 * appropriate device and potentially also negotiate newer protocol versions
 * that might come in the future.
 * 
 * The caller must provide a sufficiently large buffer to recieve the entire
 * message. If not, we return -EFBIG.
 */
ssize_t gpio_user_read_early(struct gpio_user_session *sess, char __user *buf,
			     size_t len)
{
	unsigned long copy_result;

	// Reading before initialization returns a small information
	// packet that identifies this device as being the user GPIO
	// device and identifies which userspace API version it's
	// implementing. There is currently only one version, which
	// is version zero.
	if (len < sizeof(gpio_user_header_magic))
		return -EFBIG; // Buffer is too small.
	copy_result = copy_to_user(buf, &gpio_user_header_magic,
				   sizeof(gpio_user_header_magic));
	if (copy_result != 0)
		return -EINVAL;
	return sizeof(gpio_user_header_magic);
}

/*
 * gpio_user_read_active deals with a read call for an active session.
 * In that case, the caller is looking for another message to process, which
 * could either be a new request for information about the current GPIO states
 * or it could be a cancellation of a previous request whose result is no
 * longer needed, e.g. because the requesting process has terminated.
 *
 * Read will block if there is no message pending, so a simple userspace caller
 * can use a simple loop around a blocking read. More complex callers may use
 * poll instead; see gpio_user_poll.
 */
ssize_t gpio_user_read_active(struct gpio_user_session *sess, char __user *buf,
			      size_t len)
{
	return -EINVAL;
}

/*
 * gpio_user_read is the main handler for the read system call on session
 * filehandles. Its only job is to distinguish active from uninitialized
 * sessions and dispatch to other functions accordingly.
 */
ssize_t gpio_user_read(struct file *f, char __user *buf, size_t len,
		       loff_t *offset)
{
	struct gpio_user_session *sess = f->private_data;
	ssize_t result = -EINVAL;

	mutex_lock(&sess->lock);
	if (gpio_user_session_active(sess))
		result = gpio_user_read_active(sess, buf, len);
	else
		result = gpio_user_read_early(sess, buf, len);
	mutex_unlock(&sess->lock);

	return result;
}

/*
 * gpio_user_write_early deals with a write call for an uninitialized session.
 * In that case, the only valid message is an initialization message which
 * moves the session into the initialized state.
 * 
 * The initialization message includes a "magic number" prefix that indicates
 * the caller is indeed attempting to talk our protocol (hasn't confused our
 * device with some other one) followed by data that populates the dynamic
 * struct gpio_chip that the caller will provide via the session.
 */
ssize_t gpio_user_write_early(struct gpio_user_session *sess,
			      const char __user *buf, size_t len)
{
	struct device *dev = gpio_user_miscdevice.this_device;
	struct gpio_chip *chip = &sess->chip;
	pid_t info_pid = task_pid_nr(current); // for log messages only
	struct gpio_user_init_message msg;
	unsigned long copy_result;
	int result;
	int i;

	if (len != sizeof(msg))
		return -EINVAL; // Wrong size to be an init message.

	copy_result = copy_from_user(&msg, buf, sizeof(msg));
	if (copy_result != 0)
		return -EINVAL;

	if (memcmp(&gpio_user_header_magic, msg.magic, sizeof(msg.magic)) != 0)
		return -EINVAL;

	if (msg.ngpio > 65535) {
		return -EINVAL; // kernel API uses a u16
	}

	chip->label = gpio_user_default_label;
	// If the label field contains a reasonable null-terminated
	// string then we'll use it as a custom label.
	for (i = 0; i < sizeof(msg.label); i++) {
		sess->_label[i] = msg.label[i];
		if (sess->_label[i] == 0) { // string is terminated
			if (i == 0)
				break; // ignore zero-length label
			sess->chip.label = (char *)&sess->_label;
			break;
		}
	}
	chip->ngpio = msg.ngpio;
	chip->base = -1; // base will be dynamically allocated
	chip->can_sleep = 1;
	chip->parent = dev;

	result = gpiochip_add_data(chip, sess);
	if (result != 0) {
		// The chip must appear uninitialized if init failed.
		chip->label = NULL;
		return result;
	}

	dev_info(dev, "new GPIO chip from process %d with %d signals, base %d",
		 info_pid, chip->ngpio, chip->base);

	return 0;
}

/*
 * gpio_user_write_early deals with a write call for an active session.
 * In that case, writes are either responses to earlier requests (which the
 * program received via "read") or unsolicited event notifications that
 * behave like interrupts.
 */
ssize_t gpio_user_write_active(struct gpio_user_session *sess,
			       const char __user *buf, size_t len)
{
	return -EINVAL;
}

/*
 * gpio_user_write is the main handler for the write system call on session
 * filehandles. Its only job is to distinguish active from uninitialized
 * sessions and dispatch to other functions accordingly.
 */
ssize_t gpio_user_write(struct file *f, const char __user *buf, size_t len,
			loff_t *offset)
{
	struct gpio_user_session *sess = f->private_data;
	ssize_t result = -EINVAL;

	mutex_lock(&sess->lock);
	if (gpio_user_session_active(sess))
		result = gpio_user_write_active(sess, buf, len);
	else
		result = gpio_user_write_early(sess, buf, len);
	mutex_unlock(&sess->lock);

	return result;
}

unsigned int gpio_user_poll(struct file *f, struct poll_table_struct *pt)
{
	// TODO: Indicate that the file is always writable but is readable
	// only if there is a pending message.
	return -EINVAL;
}

const struct file_operations gpio_user_operations = {
	.owner = THIS_MODULE,
	.open = gpio_user_open,
	.llseek = no_llseek,
	.read = gpio_user_read,
	.write = gpio_user_write,
	.poll = gpio_user_poll,
	.release = gpio_user_release,
};
EXPORT_SYMBOL_GPL(gpio_user_operations);

static struct miscdevice gpio_user_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usergpio",
	.fops = &gpio_user_operations,
};

static int gpio_user_probe(struct platform_device *pdev)
{
	//struct device *dev = &pdev->dev;
	return 0;
}

static void __exit gpio_user_remove_all(void)
{
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_user_dt_ids[] = {
	{},
};
MODULE_DEVICE_TABLE(of, gpio_user_dt_ids);
#endif

static struct platform_driver gpio_user_driver = {
	.probe = gpio_user_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(gpio_user_dt_ids),
	},
};

static int __init gpio_user_init(void)
{
	int err = -ENOMEM;

	err = misc_register(&gpio_user_miscdevice);
	if (err)
		return err;

	return platform_driver_register(&gpio_user_driver);
}
module_init(gpio_user_init);

static void __exit gpio_user_exit(void)
{
	misc_deregister(&gpio_user_miscdevice);
	gpio_user_remove_all();
	platform_driver_unregister(&gpio_user_driver);
}
module_exit(gpio_user_exit);

MODULE_AUTHOR("Martin Atkins <mart@degeneration.co.uk>");
MODULE_DESCRIPTION("GPIO chip drivers in userpsace");
MODULE_LICENSE("GPL v2");
