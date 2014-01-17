struct class *speaker_class;

struct speaker_class_dev {
    const char	*name;
    struct device *dev;
    int	state;
};

struct speaker_class_dev speaker_dev = {
    .name = "output",
};

static ssize_t speaker_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct speaker_class_dev *sdev = dev_get_drvdata(dev);
    return sprintf(buf, "%u\n", sdev->state);
}

static ssize_t speaker_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct speaker_class_dev *sdev = dev_get_drvdata(dev);
    
    if(hardware_mute_pin_can_action==0) return 0;   //lizhengwei 
    if (size > 0) {
        if (buf[0] == '0') {
            DBG("speaker is off!\n");
            sdev->state = 0;
            codec_set_spk(0);
        } else if (buf[0] == '1') {
            DBG("speaker is on!\n");
            sdev->state = 1;
            codec_set_spk(1);
        } else {
            DBG("invalid state!\n");
        }
    }
    return size;
}

static DEVICE_ATTR(state, 0666, speaker_state_show, speaker_state_store);

static int create_speaker_class(struct speaker_class_dev *sdev)
{
    int ret;
       hardware_mute_pin_can_action=0;  //lizhengwei
	if (!speaker_class) {
		speaker_class = class_create(THIS_MODULE, "speaker");
		if (IS_ERR(speaker_class))
			return PTR_ERR(speaker_class);
	}
	sdev->dev = device_create(speaker_class, NULL, 0, sdev, "%s", sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);
		
    sdev->state = 0;
//    codec_set_spk(0);

    ret = device_create_file(sdev->dev, &dev_attr_state);
	if (ret < 0)
		goto err_create_file;
	return 0;

err_create_file:
    device_unregister(sdev->dev);
    class_destroy(speaker_class);
    return -1;
}

static void destroy_speaker_class(struct speaker_class_dev *sdev)
{
    device_remove_file(sdev->dev, &dev_attr_state);   //文件销毁
    device_unregister(sdev->dev);                     //设备销毁
    class_destroy(speaker_class);                     //类销毁
}
